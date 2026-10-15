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

static void boutons_init(void);          // initialisation des boutons
static void boutons_traiter(void);       // traitement des boutons (anti-rebond et appuis longs)
static void anti_rebond_rouge(void);     // anti-rebond du bouton rouge
static void anti_rebond_vert(void);      // anti-rebond du bouton vert
static void verifier_appuis_longs(void); // détection des appuis longs

static void appui_long_rouge(void);      // action à effectuer lors d'un appui long sur le bouton rouge
static void appui_long_vert(void);       // action à effectuer lors d'un appui long sur le bouton vert

static void initialiser_mode_demarrage(void);  // initialisation du mode au démarrage
static void changer_mode(mode_fonctionnement_t nouveau_mode);  // changement de mode
static void afficher_couleur_mode(void);  // TODO : affichage de la couleur de la LED selon le mode actif
static void gestion_mode(void); // gestion des modes (délai de configuration, acquisitions, etc.)

static void    initialiser_systeme(void);           // capteurs, RTC, GPS, carte SD, CONFIG.TXT, revision_suivante
static uint8_t acquerir_mesures(uint8_t lire_gps);  // capteurs + RTC (+ GPS si lire_gps) , retourne 1 si horodatage valide
static void    enregistrer_sd(void);                // ligne de mesure, changement de date, archivage, écriture
static void    traiter_commande(void);              // lecture et exécution d'une commande console
static void    signaler_erreur(code_erreur_t code); // active le bit de l'erreur dans erreurs_actives
static void    effacer_erreur(code_erreur_t code);  // efface le bit de l'erreur dans erreurs_actives
static void    mettre_a_jour_led(void);             // TODO : erreur prioritaire en clignotement, sinon couleur du mode

// TODO
static void    initialiser_systeme(void);           // capteurs, RTC, GPS, carte SD, CONFIG.TXT, revision_suivante
static uint8_t acquerir_mesures(uint8_t lire_gps);  // capteurs + RTC (+ GPS si lire_gps) , retourne 1 si horodatage valide
static void    enregistrer_sd(void);                // ligne de mesure, changement de date, archivage, écriture
static void    traiter_commande(void);              // lecture et exécution d'une commande console
static void    signaler_erreur(code_erreur_t code); // active le bit de l'erreur dans erreurs_actives
static void    effacer_erreur(code_erreur_t code);  // efface le bit de l'erreur dans erreurs_actives
static void    mettre_a_jour_led(void);             // erreur prioritaire en clignotement, sinon couleur du mode

// INITIALISATION DES BOUTONS

static void boutons_init(void)
{
    GPIO_InitTypeDef gpio_init = {0}; // structure d'initialisation des GPIOs

    // Ports et broches définis dans board_pins.h
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio_init.Mode = GPIO_MODE_IT_RISING_FALLING; // interruption sur front montant et descendant
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;        // vitesse faible pour les boutons
    gpio_init.Pull = GPIO_PULLUP;                 // boutons actifs à l'état bas (appuyé = 0)

    // Bouton rouge
    gpio_init.Pin = BUTTON1_PIN;
    HAL_GPIO_Init(BUTTON1_PORT, &gpio_init); // 

    // Bouton vert
    gpio_init.Pin = BUTTON2_PIN;
    HAL_GPIO_Init(BUTTON2_PORT, &gpio_init);

    // Priorités des interruptions
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0); // PA10 -> EXTI10
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);     // PB3 -> EXTI3
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    // Boutons actifs à l'état bas : GPIO_PIN_RESET = appuyé
    rouge_etat_stable = (HAL_GPIO_ReadPin(BUTTON1_PORT, BUTTON1_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
    vert_etat_stable  = (HAL_GPIO_ReadPin(BUTTON2_PORT, BUTTON2_PIN) == GPIO_PIN_RESET) ? 1U : 0U;

    // Bouton déjà maintenu au démarrage : on lance son chronomètre
    if (rouge_etat_stable != 0U)
    {
        rouge_debut_appui     = HAL_GetTick();
        rouge_appui_demarrage = 1U;
    }

    if (vert_etat_stable != 0U)
    {
        vert_debut_appui = HAL_GetTick();
    }
}

// GESTION DES INTERRUPTIONS

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BUTTON1_PIN);
}

void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(BUTTON2_PIN);
}

// Callback HAL : appelée par HAL_GPIO_EXTI_IRQHandler() lors d'une interruption GPIO
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t maintenant = HAL_GetTick(); // instant de l'interruption

    if (GPIO_Pin == BUTTON1_PIN)  // bouton rouge
    {
        rouge_instant_front = maintenant; // mémorisation de l'instant du front
        rouge_front_detecte = 1U;
    }
    else if (GPIO_Pin == BUTTON2_PIN)  // bouton vert
    {
        vert_instant_front = maintenant; // mémorisation de l'instant du front
        vert_front_detecte = 1U;
    }
}

// ANTI-REBOND DU BOUTON ROUGE

static void anti_rebond_rouge(void)
{
    uint32_t maintenant = HAL_GetTick(); // instant actuel
    uint8_t  nouvel_etat;                // état du bouton après le front détecté

    // Aucun front ou dernier front trop récent : attente que le signal soit stable
    if ((rouge_front_detecte == 0U) || ((maintenant - rouge_instant_front) < DEBOUNCE_MS))
    {
        return;
    }

    rouge_front_detecte = 0U; // front traité : détection d'un nouveau front possible

    nouvel_etat = (HAL_GPIO_ReadPin(BUTTON1_PORT, BUTTON1_PIN) == GPIO_PIN_RESET) ? 1U : 0U; // 1 = appuyé, 0 = relâché

    if (nouvel_etat == rouge_etat_stable)
    {
        return; // rebond sans changement d'état
    }

    rouge_etat_stable = nouvel_etat;

    if (nouvel_etat != 0U)
    {
        // Début de l'appui : on démarre le chronomètre
        rouge_debut_appui     = maintenant;
        rouge_appui_long_fait = 0U;
    }
    else
    {
        // Relâchement : l'appui de démarrage (déjà pris en compte au boot) cesse d'être ignoré
        rouge_appui_demarrage = 0U;
    }
}

// ANTI-REBOND DU BOUTON VERT

static void anti_rebond_vert(void)
{
    uint32_t maintenant = HAL_GetTick();
    uint8_t  nouvel_etat;

    if ((vert_front_detecte == 0U) || ((maintenant - vert_instant_front) < DEBOUNCE_MS))
    {
        return;
    }

    vert_front_detecte = 0U;

    nouvel_etat = (HAL_GPIO_ReadPin(BUTTON2_PORT, BUTTON2_PIN) == GPIO_PIN_RESET) ? 1U : 0U;

    if (nouvel_etat == vert_etat_stable)
    {
        return;
    }

    vert_etat_stable = nouvel_etat;

    if (nouvel_etat != 0U)
    {
        vert_debut_appui     = maintenant;
        vert_appui_long_fait = 0U;
    }
}

// DETECTION DES APPUIS LONGS
// L'action se déclenche dès que LONG_PRESS_MS est atteint (bouton encore maintenu, une seule fois par appui)

static void verifier_appuis_longs(void)
{
    uint32_t maintenant = HAL_GetTick();

    if ((rouge_etat_stable != 0U) && (rouge_appui_long_fait == 0U) && (rouge_appui_demarrage == 0U)
        && ((maintenant - rouge_debut_appui) >= LONG_PRESS_MS))
    {
        rouge_appui_long_fait = 1U;
        appui_long_rouge();
    }

    if ((vert_etat_stable != 0U) && (vert_appui_long_fait == 0U)
        && ((maintenant - vert_debut_appui) >= LONG_PRESS_MS))
    {
        vert_appui_long_fait = 1U;
        appui_long_vert();
    }
}

static void boutons_traiter(void)
{
    anti_rebond_rouge();
    anti_rebond_vert();
    verifier_appuis_longs();
}

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
}

// INITIALISATION DU MODE AU DEMARRAGE

static void initialiser_mode_demarrage(void)
{
    if (rouge_etat_stable != 0U) // bouton rouge maintenu au démarrage
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

// PROGRAMME PRINCIPAL

int main(void)
{
    Global_Init();
    boutons_init();

    initialiser_mode_demarrage();

    while (1)
    {
        boutons_traiter();
        gestion_mode();

        // TODO : traitements
    }
}