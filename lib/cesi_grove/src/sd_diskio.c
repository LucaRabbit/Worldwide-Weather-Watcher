/*
 * Couche "diskio" de FatFs : accès à la carte SD en mode SPI (SD v1, v2, SDHC/SDXC).
 * FatFs appelle ces fonctions pour lire et écrire des secteurs de 512 octets.
 */

#include <stdbool.h>
#include "ff.h"
#include "diskio.h"
#include "board_pins.h"
#include "rtc_interne.h"

extern SPI_HandleTypeDef SD_SPI_HANDLE;

// COMMANDES SD (bit 7 = commande applicative ACMD, précédée de CMD55)

#define CMD0    0U            // GO_IDLE_STATE
#define CMD1    1U            // SEND_OP_COND (MMC)
#define ACMD41  (0x80U | 41U) // SEND_OP_COND (SDC)
#define CMD8    8U            // SEND_IF_COND
#define CMD9    9U            // SEND_CSD
#define CMD12   12U           // STOP_TRANSMISSION
#define CMD16   16U           // SET_BLOCKLEN
#define CMD17   17U           // READ_SINGLE_BLOCK
#define CMD24   24U           // WRITE_BLOCK
#define CMD55   55U           // APP_CMD
#define CMD58   58U           // READ_OCR

// TYPES DE CARTE

#define CT_MMC   0x01U
#define CT_SD1   0x02U
#define CT_SD2   0x04U
#define CT_BLOCK 0x08U        // adressage par bloc (SDHC/SDXC)

#define SD_TIMEOUT_INIT_MS   1000U
#define SD_TIMEOUT_LECT_MS    200U
#define SD_TIMEOUT_ECR_MS     500U
#define SD_TIMEOUT_PRET_INIT_MS 20U   // carte absente ou bloquée : abandon rapide pendant l'initialisation
#define SD_ESSAIS_CMD0          5U

static volatile DSTATUS etat = STA_NOINIT;
static uint8_t type_carte = 0U;
static uint32_t delai_pret_ms = SD_TIMEOUT_ECR_MS; // attente max de la carte avant chaque commande

// SPI BAS NIVEAU

static uint8_t spi_echanger(uint8_t octet)
{
    uint8_t recu = 0xFFU;
    (void)HAL_SPI_TransmitReceive(&SD_SPI_HANDLE, &octet, &recu, 1, 50);
    return recu;
}

static void spi_recevoir(uint8_t *tampon, uint16_t n)
{
    for (uint16_t i = 0U; i < n; i++)
        tampon[i] = spi_echanger(0xFFU);
}

static void spi_envoyer(const uint8_t *tampon, uint16_t n)
{
    for (uint16_t i = 0U; i < n; i++)
        (void)spi_echanger(tampon[i]);
}

// Vitesse lente (< 400 kHz) obligatoire pendant l'initialisation, rapide ensuite
static void spi_vitesse(uint32_t prediviseur)
{
    __HAL_SPI_DISABLE(&SD_SPI_HANDLE);
    MODIFY_REG(SD_SPI_HANDLE.Instance->CR1, SPI_CR1_BR, prediviseur);
    __HAL_SPI_ENABLE(&SD_SPI_HANDLE);
}

// SÉLECTION DE LA CARTE

static bool attendre_prete(uint32_t timeout_ms)
{
    uint32_t debut = HAL_GetTick();

    do
    {
        if (spi_echanger(0xFFU) == 0xFFU)
            return true;
    } while ((HAL_GetTick() - debut) < timeout_ms);

    return false;
}

static void deselectionner(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
    (void)spi_echanger(0xFFU); // un cycle d'horloge pour libérer MISO
}

static bool selectionner(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
    (void)spi_echanger(0xFFU);

    if (attendre_prete(delai_pret_ms))
        return true;

    deselectionner();
    return false;
}

// ENVOI D'UNE COMMANDE (retourne la réponse R1, bit 7 à 1 = pas de réponse)

static uint8_t envoyer_commande(uint8_t cmd, uint32_t arg)
{
    uint8_t reponse;

    if (cmd & 0x80U) // ACMD : CMD55 d'abord
    {
        cmd &= 0x7FU;
        reponse = envoyer_commande(CMD55, 0U);
        if (reponse > 1U) return reponse;
    }

    if (cmd != CMD12)
    {
        deselectionner();
        if (!selectionner()) return 0xFFU;
    }

    uint8_t trame[6];
    trame[0] = (uint8_t)(0x40U | cmd);
    trame[1] = (uint8_t)(arg >> 24);
    trame[2] = (uint8_t)(arg >> 16);
    trame[3] = (uint8_t)(arg >> 8);
    trame[4] = (uint8_t)arg;
    trame[5] = (cmd == CMD0) ? 0x95U : (cmd == CMD8) ? 0x87U : 0x01U; // CRC requis pour CMD0 et CMD8 uniquement
    spi_envoyer(trame, 6);

    if (cmd == CMD12) (void)spi_echanger(0xFFU); // octet de bourrage

    uint8_t n = 10U;
    do
    {
        reponse = spi_echanger(0xFFU);
    } while ((reponse & 0x80U) && --n);

    return reponse;
}

// TRANSFERT D'UN BLOC DE DONNÉES

static bool recevoir_bloc(uint8_t *tampon, uint16_t n)
{
    uint8_t jeton;
    uint32_t debut = HAL_GetTick();

    do
    {
        jeton = spi_echanger(0xFFU);
    } while (jeton == 0xFFU && (HAL_GetTick() - debut) < SD_TIMEOUT_LECT_MS);

    if (jeton != 0xFEU) return false; // jeton de début de données absent

    spi_recevoir(tampon, n);
    (void)spi_echanger(0xFFU); // CRC ignoré
    (void)spi_echanger(0xFFU);

    return true;
}

static bool envoyer_bloc(const uint8_t *tampon, uint8_t jeton)
{
    if (!attendre_prete(SD_TIMEOUT_ECR_MS)) return false;

    (void)spi_echanger(jeton);
    spi_envoyer(tampon, 512);
    (void)spi_echanger(0xFFU); // CRC factice
    (void)spi_echanger(0xFFU);

    uint8_t reponse = spi_echanger(0xFFU);
    return (reponse & 0x1FU) == 0x05U; // données acceptées
}

// FONCTIONS DISKIO (appelées par FatFs)

DSTATUS disk_status(BYTE pdrv)
{
    if (pdrv != 0U) return STA_NOINIT;
    return etat;
}

DSTATUS disk_initialize(BYTE pdrv)
{
    if (pdrv != 0U) return STA_NOINIT;

    uint8_t ocr[4];
    uint8_t type = 0U;

    spi_vitesse(SPI_BAUDRATEPRESCALER_256); // 4 MHz / 256 = 15,6 kHz
    delai_pret_ms = SD_TIMEOUT_PRET_INIT_MS;  // une carte qui répond est prête immédiatement
    HAL_Delay(10);

    // Au moins 74 cycles d'horloge, CS à l'état haut, pour réveiller la carte
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
    for (uint8_t i = 0U; i < 10U; i++) (void)spi_echanger(0xFFU);

    // Passage en mode SPI (état idle) : certaines cartes demandent plusieurs essais
    uint8_t reponse_cmd0 = 0xFFU;
    for (uint8_t essai = 0U; essai < SD_ESSAIS_CMD0 && reponse_cmd0 != 1U; essai++)
    {
        reponse_cmd0 = envoyer_commande(CMD0, 0U);
    }

    if (reponse_cmd0 == 1U)
    {
        uint32_t debut = HAL_GetTick();

        if (envoyer_commande(CMD8, 0x1AAU) == 1U) // carte SD v2
        {
            spi_recevoir(ocr, 4);

            if (ocr[2] == 0x01U && ocr[3] == 0xAAU) // carte compatible 2,7-3,6 V
            {
                // Attente de la fin d'initialisation (HCS = 1 : SDHC accepté)
                while ((HAL_GetTick() - debut) < SD_TIMEOUT_INIT_MS && envoyer_commande(ACMD41, 1UL << 30) != 0U) {}

                if ((HAL_GetTick() - debut) < SD_TIMEOUT_INIT_MS && envoyer_commande(CMD58, 0U) == 0U)
                {
                    spi_recevoir(ocr, 4);
                    type = (ocr[0] & 0x40U) ? (CT_SD2 | CT_BLOCK) : CT_SD2; // bit CCS : SDHC/SDXC
                }
            }
        }
        else // carte SD v1 ou MMC
        {
            uint8_t cmd;

            if (envoyer_commande(ACMD41, 0U) <= 1U) { type = CT_SD1; cmd = ACMD41; }
            else                                      { type = CT_MMC; cmd = CMD1;   }

            while ((HAL_GetTick() - debut) < SD_TIMEOUT_INIT_MS && envoyer_commande(cmd, 0U) != 0U) {}

            if ((HAL_GetTick() - debut) >= SD_TIMEOUT_INIT_MS || envoyer_commande(CMD16, 512U) != 0U)
                type = 0U; // taille de bloc forcée à 512 octets impossible
        }
    }

    type_carte = type;
    deselectionner();
    delai_pret_ms = SD_TIMEOUT_ECR_MS; // en fonctionnement : une écriture peut occuper la carte jusqu'à 500 ms

    if (type != 0U)
    {
        spi_vitesse(SPI_BAUDRATEPRESCALER_2); // 4 MHz / 2 = 2 MHz
        etat &= (DSTATUS)~STA_NOINIT;
    }
    else
    {
        etat = STA_NOINIT;
    }

    return etat;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0U || count == 0U) return RES_PARERR;
    if (etat & STA_NOINIT) return RES_NOTRDY;

    for (UINT i = 0U; i < count; i++)
    {
        uint32_t adresse = (uint32_t)(sector + i);
        if (!(type_carte & CT_BLOCK)) adresse *= 512U; // SDSC : adresse en octets

        if (envoyer_commande(CMD17, adresse) != 0U || !recevoir_bloc(buff + (i * 512U), 512U))
        {
            deselectionner();
            return RES_ERROR;
        }
    }

    deselectionner();
    return RES_OK;
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
    if (pdrv != 0U || count == 0U) return RES_PARERR;
    if (etat & STA_NOINIT) return RES_NOTRDY;

    for (UINT i = 0U; i < count; i++)
    {
        uint32_t adresse = (uint32_t)(sector + i);
        if (!(type_carte & CT_BLOCK)) adresse *= 512U;

        if (envoyer_commande(CMD24, adresse) != 0U || !envoyer_bloc(buff + (i * 512U), 0xFEU))
        {
            deselectionner();
            return RES_ERROR;
        }
    }

    deselectionner();
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)buff;

    if (pdrv != 0U) return RES_PARERR;
    if (etat & STA_NOINIT) return RES_NOTRDY;

    if (cmd == CTRL_SYNC) // attente de la fin de la dernière écriture
    {
        bool ok = selectionner();
        deselectionner();
        return ok ? RES_OK : RES_ERROR;
    }

    return RES_PARERR; // autres commandes inutiles (pas de formatage)
}

// Marque la carte comme non initialisée (carte retirée) : le prochain montage la réinitialisera
void disk_reinitialiser(void)
{
    etat = STA_NOINIT;
}

// HORODATAGE DES FICHIERS (date de création / modification, fournie par la RTC interne)

DWORD get_fattime(void)
{
    RTC_DateTime dt;

    if (!RTCInterne_GetDateTime(&dt))
        return ((DWORD)(2026 - 1980) << 25) | (1UL << 21) | (1UL << 16); // 01/01/2026 par défaut

    return ((DWORD)(dt.year - 1980U) << 25)
         | ((DWORD)dt.month          << 21)
         | ((DWORD)dt.day            << 16)
         | ((DWORD)dt.hours          << 11)
         | ((DWORD)dt.minutes        << 5)
         | ((DWORD)dt.seconds        >> 1);
}