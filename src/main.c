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

// CONFIGURATION

#define LONG_PRESS_MS       5000U  // appui long = 5 secondes
#define CONFIG_TIMEOUT_MS   10000U // délai d'inactivité du mode configuration (10 secondes pour tester, 1800000U = 30 min en final)
#define DEBOUNCE_MS         50U    // délai pour éviter les rebonds

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

// Bouton rouge (BUTTON1)
static volatile uint8_t  rouge_front_detecte = 0U; // écrit par l'ISR
static volatile uint32_t rouge_instant_front = 0U; // écrit par l'ISR
static uint8_t  rouge_etat_stable = 0U;            // 1 = appuyé
static uint32_t rouge_debut_appui = 0U;            // instant du début de l'appui (chronomètre)
static uint8_t  rouge_appui_demarrage = 0U;        // appui déjà présent au démarrage, ignoré jusqu'au relâchement
static uint8_t  rouge_appui_long_fait = 0U;        // action déjà déclenchée pour l'appui en cours

// Bouton vert (BUTTON2)
static volatile uint8_t  vert_front_detecte = 0U;
static volatile uint32_t vert_instant_front = 0U;
static uint8_t  vert_etat_stable = 0U;
static uint32_t vert_debut_appui = 0U;
static uint8_t  vert_appui_long_fait = 0U;

// Délai d'inactivité du mode configuration
static uint32_t config_derniere_activite = 0U;

// PROTOTYPES DES FONCTIONS
// TODO

static void boutons_init(void);          // initialisation des boutons
static void boutons_traiter(void);       // traitement des boutons (anti-rebond et appuis longs)
static void anti_rebond_rouge(void);     // anti-rebond du bouton rouge
static void anti_rebond_vert(void);      // anti-rebond du bouton vert
static void verifier_appuis_longs(void); // détection des appuis longs

static void appui_long_rouge(void);      // action à effectuer lors d'un appui long sur le bouton rouge
static void appui_long_vert(void);       // action à effectuer lors d'un appui long sur le bouton vert

static void initialiser_mode_demarrage(void);  // initialisation du mode au démarrage
static void changer_mode(mode_fonctionnement_t nouveau_mode);  // changement de mode
static void afficher_couleur_mode(void);  // affichage de la couleur de la LED selon le mode actif
static void gestion_mode(void); // gestion des modes (délai de configuration, acquisitions, etc.)

static void    initialiser_systeme(void);           // capteurs, RTC, GPS, carte SD, CONFIG.TXT, revision_suivante
static uint8_t acquerir_mesures(uint8_t lire_gps);  // capteurs + RTC (+ GPS si lire_gps) , retourne 1 si horodatage valide
static void    enregistrer_sd(void);                // ligne de mesure, changement de date, archivage, écriture
static void    traiter_commande(void);              // lecture et exécution d'une commande console
static void    signaler_erreur(code_erreur_t code); // active le bit de l'erreur dans erreurs_actives
static void    effacer_erreur(code_erreur_t code);  // efface le bit de l'erreur dans erreurs_actives
static void    mettre_a_jour_led(void);             // erreur prioritaire en clignotement, sinon couleur du mode

int main(void)
{
    Global_Init();

    while (1)
    {
        
    }
}