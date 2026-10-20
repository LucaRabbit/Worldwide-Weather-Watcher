#include <stdio.h>
#include <stdint.h>
#include "setup.h"
#include "cesi_types.h"
#include "grove_rgb.h"
#include "grove_button.h"
#include "grove_light.h"
#include "rtc_interne.h"
#include "grove_gps_air530z.h"
#include "grove_bme280.h"
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

#define LOG_INTERVAL_DEFAUT   10000U    // intervalle entre deux mesures (min)
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

#define GPS_ATTENTE_MS      2000U        // attente max d'une trame GGA (le GPS en émet une par seconde)
#define LUMIN_ERREUR        0xFFFFU      // valeur renvoyée par GroveLight_ReadRaw() en cas d'échec
#define PERIODE_MAINTENANCE_MS  2000U    // rafraîchissement de l'affichage en maintenance (ms)

#define TAILLE_MOYENNE      5U           // nombre de mesures dans la moyenne glissante

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

// CAPTEURS (index des compteurs de timeouts, NB_CAPTEURS = 4)
 
typedef enum
{
    CAPT_TEMPERATURE,
    CAPT_HUMIDITE,
    CAPT_PRESSION,
    CAPT_LUMINOSITE
} capteur_t;

// MOYENNE GLISSANTE (tampon circulaire des TAILLE_MOYENNE dernières mesures valides)
 
typedef struct
{
    float   valeurs[TAILLE_MOYENNE];
    uint8_t index;      // case où sera écrite la prochaine valeur
    uint8_t nombre;     // nombre de valeurs présentes (< TAILLE_MOYENNE au démarrage)
} moyenne_t;

// PARAMÈTRES (valeurs par défaut au démarrage, modifiables plus tard en mode configuration)
 
typedef struct
{
    uint16_t log_interval;   // min
    uint16_t file_max_size;  // octets
    uint16_t timeout;        // s
    uint8_t  lumin;
    uint16_t lumin_low;
    uint16_t lumin_high;
    uint8_t  temp_air;
    int8_t   min_temp_air;
    int8_t   max_temp_air;
    uint8_t  hygr;
    int8_t   hygr_mint;
    int8_t   hygr_maxt;
    uint8_t  pressure;
    uint16_t pressure_min;
    uint16_t pressure_max;
} parametres_t;

// MESURES (dernière acquisition ; un champ *_ok à false = valeur NA)
 
typedef struct
{
    RTC_DateTime horodatage;
    bool     temperature_ok;
    bool     humidite_ok;
    bool     pression_ok;
    bool     luminosite_ok;
    bool     gps_ok;
    float    temperature_c;
    float    humidite_pct;
    float    pression_hpa;
    uint16_t luminosite;     // 0-1023
    double   latitude;
    double   longitude;
} mesures_t;

// VARIABLES GLOBALES

static mode_fonctionnement_t mode_actuel    = MODE_STANDARD;
static mode_fonctionnement_t mode_precedent = MODE_STANDARD; // mode à restaurer en sortie de maintenance
static parametres_t params =
{
    .log_interval  = LOG_INTERVAL_DEFAUT,
    .file_max_size = FILE_MAX_SIZE_DEFAUT,
    .timeout       = TIMEOUT_DEFAUT,
    .lumin         = LUMIN_DEFAUT,
    .lumin_low     = LUMIN_LOW_DEFAUT,
    .lumin_high    = LUMIN_HIGH_DEFAUT,
    .temp_air      = TEMP_AIR_DEFAUT,
    .min_temp_air  = MIN_TEMP_AIR_DEFAUT,
    .max_temp_air  = MAX_TEMP_AIR_DEFAUT,
    .hygr          = HYGR_DEFAUT,
    .hygr_mint     = HYGR_MINT_DEFAUT,
    .hygr_maxt     = HYGR_MAXT_DEFAUT,
    .pressure      = PRESSURE_DEFAUT,
    .pressure_min  = PRESSURE_MIN_DEFAUT,
    .pressure_max  = PRESSURE_MAX_DEFAUT
};
static mesures_t mesures;                            // dernière acquisition (lue par enregistrer_sd)
static uint8_t   erreurs_actives = 0U;               // 1 bit par code_erreur_t
static uint8_t   nb_timeouts[NB_CAPTEURS] = {0U};    // timeouts consécutifs par capteur
static uint8_t   nb_timeouts_gps = 0U;               // timeouts consécutifs du GPS

static moyenne_t moy_temperature;
static moyenne_t moy_humidite;
static moyenne_t moy_pression;
static moyenne_t moy_luminosite;

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
static void    afficher_mesures(void);              // affichage série des mesures (mode maintenance ou test capteurs)
static void    enregistrer_sd(void);                // ligne de mesure, changement de date, archivage, écriture
static void    traiter_commande(void);              // lecture et exécution d'une commande console
static void    signaler_erreur(code_erreur_t code); // active le bit de l'erreur dans erreurs_actives
static void    effacer_erreur(code_erreur_t code);  // efface le bit de l'erreur dans erreurs_actives
static void    mettre_a_jour_led(void);             // erreur prioritaire en clignotement, sinon couleur du mode

static void mode_standard_ou_eco(uint8_t facteur);  // acquisition + SD, facteur = 1 (standard) ou FACTEUR_ECO
static void mode_maintenance(void);                 // acquisition + affichage série, sans SD
static void mode_configuration(void);               // lecture et exécution d'une commande console, délai d'inactivité

// MODE STANDARD / ÉCONOMIQUE

static void mode_standard_ou_eco(uint8_t facteur)
{
    static uint32_t derniere_mesure = 0U;
    static uint8_t  compteur_gps = 0U;
    static bool     premiere_mesure = true;

    uint32_t intervalle_ms = (uint32_t)params.log_interval * 60000U * facteur;

    // Première mesure immédiate, puis une mesure par intervalle
    if (!premiere_mesure && (HAL_GetTick() - derniere_mesure) < intervalle_ms)
    {
        return;
    }
    premiere_mesure = false;
    derniere_mesure = HAL_GetTick();

    // En éco, GPS lu 1 cycle sur FACTEUR_ECO ; en standard (facteur = 1), à chaque cycle
    uint8_t lire_gps = ((compteur_gps % facteur) == 0U) ? 1U : 0U;
    compteur_gps++;

    acquerir_mesures(lire_gps);
    // TODO: enregistrer_sd();
}

// MODE MAINTENANCE

static void mode_maintenance(void)
{
    static uint32_t derniere_mesure = 0U;

    if ((HAL_GetTick() - derniere_mesure) < PERIODE_MAINTENANCE_MS)
    {
        return;
    }
    derniere_mesure = HAL_GetTick();

    acquerir_mesures(1U);  // GPS toujours lu en maintenance
    afficher_mesures();    // affichage série, pas d'écriture SD (carte retirable)
}

// MODE CONFIGURATION

static void mode_configuration(void)
{
    // traiter_commande(); // TODO : lecture et exécution d'une commande console

    if ((HAL_GetTick() - config_derniere_activite) >= CONFIG_TIMEOUT_MS)
    {
        printf("Fin du delai de configuration\r\n");
        changer_mode(MODE_STANDARD);
    }
}

// MOYENNE GLISSANTE
 
// Ajoute une valeur (remplace la plus ancienne si le tampon est plein) et retourne la moyenne
static float moyenne_ajouter(moyenne_t *m, float valeur)
{
    m->valeurs[m->index] = valeur;
    m->index = (uint8_t)((m->index + 1U) % TAILLE_MOYENNE);
 
    if (m->nombre < TAILLE_MOYENNE)
    {
        m->nombre++;
    }
 
    float somme = 0.0f;
    for (uint8_t i = 0U; i < m->nombre; i++)
    {
        somme += m->valeurs[i];
    }
 
    return somme / (float)m->nombre;
}
 
// Vide le tampon (mesure NA, changement de mode)
static void moyenne_vider(moyenne_t *m)
{
    m->index  = 0U;
    m->nombre = 0U;
}
 
static void reinitialiser_moyennes(void)
{
    moyenne_vider(&moy_temperature);
    moyenne_vider(&moy_humidite);
    moyenne_vider(&moy_pression);
    moyenne_vider(&moy_luminosite);
}

// GESTION DES ERREURS
 
static void signaler_erreur(code_erreur_t code)
{
    erreurs_actives |= (uint8_t)(1U << code);
}
 
static void effacer_erreur(code_erreur_t code)
{
    erreurs_actives &= (uint8_t)~(1U << code);
}
 
// Met à jour le compteur de timeouts consécutifs d'un capteur
static void compter_reponse(capteur_t capteur, bool reponse)
{
    if (reponse)
    {
        nb_timeouts[capteur] = 0U;
    }
    else if (nb_timeouts[capteur] < UINT8_MAX)
    {
        nb_timeouts[capteur]++;
    }
}
 
// ACQUISITION DES MESURES
 
static uint8_t acquerir_mesures(uint8_t lire_gps)
{
    bool incoherent = false;
 
    // Toutes les valeurs à NA par défaut
    mesures.temperature_ok = false;
    mesures.humidite_ok    = false;
    mesures.pression_ok    = false;
    mesures.luminosite_ok  = false;
    mesures.gps_ok         = false;
 
    // 1. Horodatage
    uint8_t horodatage_valide = RTCInterne_GetDateTime(&mesures.horodatage) ? 1U : 0U;
 
    if (horodatage_valide)
        effacer_erreur(ERR_RTC);
    else
        signaler_erreur(ERR_RTC);
 
    // 2. BME280 : une seule lecture fournit température, humidité et pression
    if (params.temp_air || params.hygr || params.pressure)
    {
        Env_Data env;
        bool reponse = GroveBME280_ReadEnvironment(&env);
 
        if (!reponse)
        {
            (void)GroveBME280_Init(); // nouvelle tentative au prochain cycle (capteur rebranché)
        }
 
        // Température
        if (params.temp_air)
        {
            compter_reponse(CAPT_TEMPERATURE, reponse);
 
            if (reponse)
            {
                if (env.temperature_c < params.min_temp_air || env.temperature_c > params.max_temp_air)
                {
                    incoherent = true; // valeur rejetée (reste NA)
                }
                else
                {
                    mesures.temperature_c  = moyenne_ajouter(&moy_temperature, env.temperature_c);
                    mesures.temperature_ok = true;
                }
            }
        }
        else
        {
            nb_timeouts[CAPT_TEMPERATURE] = 0U;
        }
 
        // Humidité : ignorée si la température sort de [HYGR_MINT ; HYGR_MAXT]
        if (params.hygr)
        {
            compter_reponse(CAPT_HUMIDITE, reponse);
 
            if (reponse && env.temperature_c >= params.hygr_mint && env.temperature_c <= params.hygr_maxt) // valeur rejetée si température hors plage
            {
                mesures.humidite_pct = moyenne_ajouter(&moy_humidite, env.humidity_percent);
                mesures.humidite_ok  = true;
            }
        }
        else
        {
            nb_timeouts[CAPT_HUMIDITE] = 0U;
        }
 
        // Pression
        if (params.pressure)
        {
            compter_reponse(CAPT_PRESSION, reponse);
 
            if (reponse)
            {
                if (env.pressure_hpa < params.pressure_min || env.pressure_hpa > params.pressure_max)
                {
                    incoherent = true;
                }
                else
                {
                    mesures.pression_hpa = moyenne_ajouter(&moy_pression, env.pressure_hpa);
                    mesures.pression_ok  = true;
                }
            }
        }
        else
        {
            nb_timeouts[CAPT_PRESSION] = 0U;
        }
    }
    else
    {
        nb_timeouts[CAPT_TEMPERATURE] = 0U;
        nb_timeouts[CAPT_HUMIDITE]    = 0U;
        nb_timeouts[CAPT_PRESSION]    = 0U;
    }
 
    // 3. Luminosité (0-1023)
    if (params.lumin)
    {
        uint16_t lumiere = GroveLight_ReadRaw(); // 0-1023 (ADC 10 bits)
        bool reponse = (lumiere != LUMIN_ERREUR);
 
        compter_reponse(CAPT_LUMINOSITE, reponse);
 
        if (reponse)
        {
            mesures.luminosite    = moyenne_ajouter(&moy_luminosite, (float)lumiere);
            mesures.luminosite_ok = true;
        }
    }
    else
    {
        nb_timeouts[CAPT_LUMINOSITE] = 0U;
    }

    // Mesure NA : historique interrompu, la moyenne repart de zéro
    if (!mesures.temperature_ok) moyenne_vider(&moy_temperature);
    if (!mesures.humidite_ok)    moyenne_vider(&moy_humidite);
    if (!mesures.pression_ok)    moyenne_vider(&moy_pression);
    if (!mesures.luminosite_ok)  moyenne_vider(&moy_luminosite);
 
    // 4. GPS (en mode économique : un cycle sur deux)
    if (lire_gps)
    {
        GPS_Data gps = {0};
 
        if (GroveGPS_ReadGGA(&gps, GPS_ATTENTE_MS))
        {
            nb_timeouts_gps = 0U;
 
            if (gps.valid) // trame reçue mais sans fix : position NA, pas d'erreur
            {
                mesures.latitude  = gps.latitude;
                mesures.longitude = gps.longitude;
                mesures.gps_ok    = true;
            }
        }
        else if (nb_timeouts_gps < UINT8_MAX)
        {
            nb_timeouts_gps++;
        }
 
        if (nb_timeouts_gps >= NB_TIMEOUTS_ERREUR)
            signaler_erreur(ERR_GPS);
        else
            effacer_erreur(ERR_GPS);
    }
 
    // 5. Erreur capteur : au moins un capteur sans réponse NB_TIMEOUTS_ERREUR fois de suite
    bool erreur_capteur = false;
 
    for (uint8_t i = 0U; i < NB_CAPTEURS; i++)
    {
        if (nb_timeouts[i] >= NB_TIMEOUTS_ERREUR)
        {
            erreur_capteur = true;
        }
    }
 
    if (erreur_capteur)
        signaler_erreur(ERR_CAPTEUR);
    else
        effacer_erreur(ERR_CAPTEUR);
 
    // 6. Donnée incohérente
    if (incoherent)
        signaler_erreur(ERR_INCOHERENT);
    else
        effacer_erreur(ERR_INCOHERENT);
 
    return horodatage_valide;
}
 
// AFFICHAGE DES MESURES (mode maintenance ou test capteurs)
 
static void afficher_mesures(void)
{
    printf("%02u/%02u/%04u %02u:%02u:%02u | ",
           mesures.horodatage.day, mesures.horodatage.month, mesures.horodatage.year,
           mesures.horodatage.hours, mesures.horodatage.minutes, mesures.horodatage.seconds);
 
    if (mesures.temperature_ok) printf("T=%.1f C | ", mesures.temperature_c);
    else                        printf("T=" VALEUR_NA " | ");
 
    if (mesures.humidite_ok)    printf("H=%.1f %% | ", mesures.humidite_pct);
    else                        printf("H=" VALEUR_NA " | ");
 
    if (mesures.pression_ok)    printf("P=%.1f hPa | ", mesures.pression_hpa);
    else                        printf("P=" VALEUR_NA " | ");
 
    if (mesures.luminosite_ok)  printf("L=%u | ", mesures.luminosite);
    else                        printf("L=" VALEUR_NA " | ");
 
    if (mesures.gps_ok)         printf("GPS=%.6f,%.6f", mesures.latitude, mesures.longitude);
    else                        printf("GPS=" VALEUR_NA);
 
    printf(" | erreurs=0x%02X\r\n", erreurs_actives);
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
    reinitialiser_moyennes(); // pas de mélange entre les mesures de deux modes

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
    switch (mode_actuel)
    {
        case MODE_STANDARD:      mode_standard_ou_eco(1U);          break;
        case MODE_ECONOMIQUE:    mode_standard_ou_eco(FACTEUR_ECO); break;
        case MODE_MAINTENANCE:   mode_maintenance();                break;
        case MODE_CONFIGURATION: mode_configuration();              break;
        default:                                                    break;
    }
}

// PROGRAMME PRINCIPAL

int main(void)
{
    Global_Init();
    GroveRGB_Init();
    GroveButton_Init();
    GroveLight_Init();
    GroveBME280_Init();
    GroveGPS_Init();
    RTCInterne_Init();

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

    }
}