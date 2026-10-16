#include <stdio.h>
#include <stdint.h>
#include "setup.h"
#include "cesi_types.h"
#include "grove_rgb.h"
#include "grove_button.h"
#include "grove_light.h"
#include "grove_rtc_ds1307.h"
#include "grove_gps_air530z.h"
#include "grove_bme680.h"
#include "sd_logger.h"
#include "board_pins.h"

// CONFIGURATION

// LONG_PRESS_MS et DEBOUNCE_MS : définis dans grove_button.h
#define CONFIG_TIMEOUT_MS   10000U // délai d'inactivité du mode configuration (10 secondes pour tester, 1800000U = 30 min en final)

#define VERSION_PROGRAMME   "1.0.0"      // affiché par la commande VERSION
#define NUMERO_LOT          "LOT-001"    // affiché par la commande VERSION
#define NB_TIMEOUTS_ERREUR  2U           // timeouts consécutifs avant erreur capteur
#define FACTEUR_ECO         2U           // éco : intervalle x2, GPS 1 cycle sur 2
#define NB_CAPTEURS         4U           // température, humidité, pression, luminosité
#define VALEUR_NA           "NA"         // valeur écrite pour une mesure indisponible
#define FICHIER_CONFIG      "CONFIG.TXT" // fichier de configuration, racine de la carte SD
#define TAILLE_LIGNE        128U         // TODO : tampon d'une ligne de mesures (>= 64 octets, à ajuster)
#define TAILLE_COMMANDE     64U          // TODO : tampon d'une commande console (à ajuster)

// VALEURS PAR DÉFAUT DES PARAMÈTRES (modifiables en mode configuration)
// U = unsigned int

#define LOG_INTERVAL_DEFAUT   10U    // intervalle entre deux mesures (min)
#define FILE_MAX_SIZE_DEFAUT  2048U  // taille max d'un fichier de log (octets)
#define TIMEOUT_DEFAUT        30U    // abandon de l'acquisition d'un capteur (s)
#define LUMIN_DEFAUT          1U     // capteur de luminosité activé
#define LUMIN_LOW_DEFAUT      255U   // seuil luminosité faible (0-1023)
#define LUMIN_HIGH_DEFAUT     768U   // seuil luminosité forte (0-1023)
#define TEMP_AIR_DEFAUT       1U     // température de l'air activée
#define MIN_TEMP_AIR_DEFAUT   (-10)  // seuil bas de cohérence (°C)
#define MAX_TEMP_AIR_DEFAUT   60     // seuil haut de cohérence (°C)
#define HYGR_DEFAUT           1U     // hygrométrie activée
#define HYGR_MINT_DEFAUT      0      // hygrométrie ignorée en dessous (°C)
#define HYGR_MAXT_DEFAUT      50     // hygrométrie ignorée au-dessus (°C)
#define PRESSURE_DEFAUT       1U     // pression atmosphérique activée
#define PRESSURE_MIN_DEFAUT   850U   // seuil bas de cohérence (hPa)
#define PRESSURE_MAX_DEFAUT   1080U  // seuil haut de cohérence (hPa)

#define PERIODE_TEST_MS     2000U        // période d'affichage des mesures de test (ms)

// CODES D'ERREUR (rangés par priorité croissante : 1 bit par code dans erreurs_actives)

typedef enum
{
    ERR_GPS,
    ERR_CAPTEUR,
    ERR_INCOHERENT,
    ERR_RTC,
    ERR_SD_PLEINE,
    ERR_SD_ACCES
} code_erreur_t;

// MODES DE FONCTIONNEMENT

typedef enum
{
    MODE_STANDARD,
    MODE_ECONOMIQUE,
    MODE_MAINTENANCE,
    MODE_CONFIGURATION
} mode_fonctionnement_t;

// VARIABLES GLOBALES

static mode_fonctionnement_t mode_actuel    = MODE_STANDARD;
static mode_fonctionnement_t mode_precedent = MODE_STANDARD; // mode à restaurer en sortie de maintenance

// Délai d'inactivité du mode configuration
static uint32_t config_derniere_activite = 0U;

// PROTOTYPES DES FONCTIONS

static void appui_long_rouge(void);      // action à effectuer lors d'un appui long sur le bouton rouge
static void appui_long_vert(void);       // action à effectuer lors d'un appui long sur le bouton vert

static void initialiser_mode_demarrage(void);  // initialisation du mode au démarrage
static void changer_mode(mode_fonctionnement_t nouveau_mode);  // changement de mode
static void afficher_couleur_mode(void);  // affichage de la couleur de la LED selon le mode actif
static void gestion_mode(void); // gestion des modes (délai de configuration, acquisitions, etc.)

// TODO
static void    initialiser_systeme(void);           // capteurs, RTC, GPS, carte SD, CONFIG.TXT, revision_suivante
static uint8_t acquerir_mesures(uint8_t lire_gps);  // capteurs + RTC (+ GPS si lire_gps) , retourne 1 si horodatage valide
static void    enregistrer_sd(void);                // ligne de mesure, changement de date, archivage, écriture
static void    traiter_commande(void);              // lecture et exécution d'une commande console
static void    signaler_erreur(code_erreur_t code); // active le bit de l'erreur dans erreurs_actives
static void    effacer_erreur(code_erreur_t code);  // efface le bit de l'erreur dans erreurs_actives
static void    mettre_a_jour_led(void);             // erreur prioritaire en clignotement, sinon couleur du mode

// ACTIONS DU BOUTON ROUGE (appui long)

static void appui_long_rouge(void)
{
    switch (mode_actuel)
    {
        case MODE_STANDARD:
        case MODE_ECONOMIQUE:
            mode_precedent = mode_actuel;
            changer_mode(MODE_MAINTENANCE);
            break;

        case MODE_MAINTENANCE:
            changer_mode(mode_precedent);
            break;

        case MODE_CONFIGURATION:
            config_derniere_activite = HAL_GetTick();
            break;

        default:
            break;
    }
}

// ACTIONS DU BOUTON VERT (appui long)

static void appui_long_vert(void)
{
    if (mode_actuel == MODE_STANDARD)
    {
        changer_mode(MODE_ECONOMIQUE);
    }
    else if (mode_actuel == MODE_ECONOMIQUE)
    {
        changer_mode(MODE_STANDARD);
    }
}

// COULEUR DE LA LED SELON LE MODE

static void afficher_couleur_mode(void)
{
    switch (mode_actuel)
    {
        case MODE_STANDARD:      GroveRGB_SetColor(GROVE_RGB_GREEN);  break; // vert continu
        case MODE_CONFIGURATION: GroveRGB_SetColor(GROVE_RGB_YELLOW); break; // jaune continu
        case MODE_ECONOMIQUE:    GroveRGB_SetColor(GROVE_RGB_BLUE);   break; // bleu continu
        case MODE_MAINTENANCE:   GroveRGB_SetColor(GROVE_RGB_ORANGE); break; // orange continu
        default:                 GroveRGB_Off();                      break;
    }
}

// CHANGEMENT DE MODE

static void changer_mode(mode_fonctionnement_t nouveau_mode)
{
    if (nouveau_mode == mode_actuel)
    {
        return;
    }

    mode_actuel = nouveau_mode;

    switch (mode_actuel)
    {
        case MODE_STANDARD:
            printf("Mode standard actif\r\n");
            break;

        case MODE_ECONOMIQUE:
            printf("Mode economique actif\r\n");
            break;

        case MODE_MAINTENANCE:
            printf("Mode maintenance actif\r\n");
            break;

        case MODE_CONFIGURATION:
            printf("Mode configuration actif\r\n");
            config_derniere_activite = HAL_GetTick(); // chronomètre d'inactivité démarré
            break;

        default:
            break;
    }

    afficher_couleur_mode(); // mise à jour de la couleur de la LED
}

// INITIALISATION DU MODE AU DEMARRAGE

static void initialiser_mode_demarrage(void)
{
    if (GroveButton1_IsPressed()) // bouton rouge maintenu au démarrage
    {
        mode_actuel = MODE_CONFIGURATION;
        config_derniere_activite = HAL_GetTick();
        printf("Demarrage en mode configuration\r\n");
    }
    else
    {
        mode_actuel = MODE_STANDARD;
        printf("Demarrage en mode standard\r\n");
    }

    mode_precedent = MODE_STANDARD;
    afficher_couleur_mode();
}

// GESTION DES MODES

static void gestion_mode(void)
{
    if (mode_actuel == MODE_CONFIGURATION)
    {
        if ((HAL_GetTick() - config_derniere_activite) >= CONFIG_TIMEOUT_MS)
        {
            printf("Fin du delai de configuration\r\n");
            changer_mode(MODE_STANDARD);
        }
    }

    // TODO : integration acquisitions des capteurs, enregistrement sur carte SD et autres traitements selon mode actif

}

// TEST DES CAPTEURS (affichage port série)

static void test_capteurs(void)
{
    static uint32_t dernier_affichage = 0U;

    if ((HAL_GetTick() - dernier_affichage) < PERIODE_TEST_MS)
    {
        return;
    }
    dernier_affichage = HAL_GetTick();

    uint16_t lumiere = GroveLight_ReadRaw();

    if (lumiere == 0xFFFFU)
        printf("[LUM] erreur de lecture ADC\r\n");
    else
        printf("[LUM] %u / 4095\r\n", lumiere);
}

// PROGRAMME PRINCIPAL

int main(void)
{
    Global_Init();
    GroveRGB_Init();
    GroveButton_Init();
    GroveLight_Init();

    initialiser_mode_demarrage();

    while (1)
    {
        GroveButton_Process();

        if (GroveButton1_LongPress())
        {
            appui_long_rouge();
        }

        if (GroveButton2_LongPress())
        {
            appui_long_vert();
        }

        gestion_mode();

        test_capteurs();
    }
}