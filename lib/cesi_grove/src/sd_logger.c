#include "sd_logger.h"
#include "ff.h"
#include "stm32l4xx_hal.h"
#include <string.h>

#define SD_DELAI_REMONTAGE_MS 2000U // carte absente : une tentative de remontage au plus toutes les 2 s

void disk_reinitialiser(void); // sd_diskio.c

static FATFS systeme_fichiers;
static bool  montee = false;
static bool     tentative_faite = false;
static uint32_t derniere_tentative = 0U;

// FONCTIONS INTERNES

// Erreur matérielle (carte retirée, illisible) : la carte devra être remontée
static bool erreur_materielle(FRESULT res)
{
    return res == FR_DISK_ERR || res == FR_NOT_READY || res == FR_NO_FILESYSTEM
        || res == FR_INT_ERR  || res == FR_NOT_ENABLED || res == FR_INVALID_OBJECT;
}

static void demonter(void)
{
    (void)f_mount(0, "", 0);
    montee = false;
}

static SDResultat traiter_erreur(FRESULT res)
{
    if (res == FR_OK) return SD_OK;

    if (erreur_materielle(res))
        demonter();

    // FR_DENIED : création impossible (répertoire ou carte pleine)
    return (res == FR_DENIED) ? SD_ERR_PLEINE : SD_ERR_ACCES;
}

// Monte la carte si nécessaire (première utilisation ou carte réinsérée).
// Si la carte est absente, on ne retente pas à chaque appel : chaque tentative bloque le programme.
static bool assurer_montage(void)
{
    if (montee) return true;

    if (tentative_faite && (HAL_GetTick() - derniere_tentative) < SD_DELAI_REMONTAGE_MS)
        return false;

    return SDLogger_Init();
}

// FONCTIONS PUBLIQUES

bool SDLogger_Init(void)
{
    demonter();
    disk_reinitialiser();

    tentative_faite = true;
    derniere_tentative = HAL_GetTick();

    montee = (f_mount(&systeme_fichiers, "", 1) == FR_OK); // 1 : montage immédiat (détecte l'absence de carte)
    return montee;
}

bool SDLogger_EstPrete(void)
{
    return montee;
}

SDResultat SDLogger_AjouterLigne(const char *nom, const char *ligne)
{
    if (nom == 0 || ligne == 0) return SD_ERR_ACCES;
    if (!assurer_montage()) return SD_ERR_ACCES;

    FIL fichier;
    FRESULT res = f_open(&fichier, nom, FA_WRITE | FA_OPEN_APPEND);
    if (res != FR_OK) return traiter_erreur(res);

    FSIZE_t taille_initiale = f_size(&fichier);
    UINT longueur = (UINT)strlen(ligne);
    UINT ecrits1 = 0U, ecrits2 = 0U;

    res = f_write(&fichier, ligne, longueur, &ecrits1);
    if (res == FR_OK) res = f_write(&fichier, "\r\n", 2U, &ecrits2);

    if (res == FR_OK && (ecrits1 < longueur || ecrits2 < 2U))
    {
        // Carte pleine : FatFs n'a écrit qu'une partie, on retire la ligne incomplète
        (void)f_lseek(&fichier, taille_initiale);
        (void)f_truncate(&fichier);
        (void)f_close(&fichier);
        return SD_ERR_PLEINE;
    }

    FRESULT res_fermeture = f_close(&fichier); // écrit réellement les données sur la carte
    if (res == FR_OK) res = res_fermeture;

    return traiter_erreur(res);
}

int32_t SDLogger_TailleFichier(const char *nom)
{
    if (nom == 0 || !assurer_montage()) return -1;

    FILINFO info;
    FRESULT res = f_stat(nom, &info);

    if (res != FR_OK)
    {
        (void)traiter_erreur(res);
        return -1;
    }

    return (int32_t)info.fsize;
}

bool SDLogger_Existe(const char *nom)
{
    return SDLogger_TailleFichier(nom) >= 0;
}

SDResultat SDLogger_Renommer(const char *ancien, const char *nouveau)
{
    if (ancien == 0 || nouveau == 0) return SD_ERR_ACCES;
    if (!assurer_montage()) return SD_ERR_ACCES;

    return traiter_erreur(f_rename(ancien, nouveau));
}

bool SDLogger_LireLignes(const char *nom, void (*traiter)(const char *ligne))
{
    if (nom == 0 || traiter == 0 || !assurer_montage()) return false;

    FIL fichier;
    FRESULT res = f_open(&fichier, nom, FA_READ);

    if (res != FR_OK)
    {
        (void)traiter_erreur(res);
        return false;
    }

    char ligne[SD_TAILLE_LIGNE_MAX];

    while (f_gets(ligne, (int)sizeof(ligne), &fichier) != 0)
    {
        // Suppression de la fin de ligne ("\n" ou "\r\n")
        size_t n = strlen(ligne);
        while (n > 0U && (ligne[n - 1U] == '\n' || ligne[n - 1U] == '\r'))
        {
            ligne[--n] = '\0';
        }

        traiter(ligne);
    }

    bool erreur_lecture = (f_error(&fichier) != 0);
    (void)f_close(&fichier);

    if (erreur_lecture) demonter();
    return !erreur_lecture;
}

SDResultat SDLogger_Supprimer(const char *nom)
{
    if (nom == 0) return SD_ERR_ACCES;
    if (!assurer_montage()) return SD_ERR_ACCES;

    FRESULT res = f_unlink(nom);
    if (res == FR_NO_FILE) return SD_OK; // déjà absent : rien à faire

    return traiter_erreur(res);
}

bool SDLogger_EspaceLibre(uint32_t *ko_libres)
{
    if (ko_libres == 0 || !assurer_montage()) return false;

    DWORD clusters_libres;
    FATFS *fs;
    FRESULT res = f_getfree("", &clusters_libres, &fs);

    if (res != FR_OK)
    {
        (void)traiter_erreur(res);
        return false;
    }

    // Un secteur = 512 octets, donc 2 secteurs = 1 kio
    *ko_libres = (uint32_t)((clusters_libres * fs->csize) / 2U);
    return true;
}