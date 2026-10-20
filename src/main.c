#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "setup.h"
#include "cesi_types.h"
#include "grove_rgb.h"
#include "grove_button.h"
#include "grove_light.h"
#include "rtc_interne.h"
#include "grove_gps_air530z.h"
#include "grove_bme280.h"
#include "sd_logger.h"
#include "console.h"
#include "board_pins.h"

// CONFIGURATION

// LONG_PRESS_MS et DEBOUNCE_MS : définis dans grove_button.h
#define CONFIG_TIMEOUT_MS   60000U       // délai d'inactivité du mode configuration (10 s pour tester, 1800000U = 30 min en final)

#define VERSION_PROGRAMME   "1.0.0"      // affiché par la commande VERSION
#define NUMERO_LOT          "LOT-001"    // affiché par la commande VERSION
#define NB_TIMEOUTS_ERREUR  2U           // timeouts consécutifs avant erreur capteur
#define FACTEUR_ECO         2U           // éco : intervalle x2, GPS 1 cycle sur 2
#define NB_CAPTEURS         4U           // température, humidité, pression, luminosité
#define VALEUR_NA           "NA"         // valeur écrite pour une mesure indisponible
#define FICHIER_CONFIG      "CONFIG.TXT" // fichier de configuration, racine de la carte SD
#define FICHIER_CONFIG_TMP  "CONFIG.TMP" // écriture dans un fichier temporaire, puis renommage
#define TAILLE_LIGNE        128U         // tampon d'une ligne de mesures (une ligne fait environ 60 caractères)
#define TAILLE_COMMANDE     64U          // tampon d'une commande console

#define GPS_ATTENTE_MS      2000U        // âge max d'une trame GGA (le GPS en émet une par seconde)
#define LUMIN_ERREUR        0xFFFFU      // valeur renvoyée par GroveLight_ReadRaw() en cas d'échec
#define LED_PERIODE_MS      1000U        // période du clignotement d'erreur (1 Hz)
#define PERIODE_MAINTENANCE_MS  2000U    // rafraîchissement de l'affichage en maintenance (ms)

#define TAILLE_MOYENNE      5U           // nombre de mesures dans la moyenne glissante

#define REVISION_MAX        9U           // noms 8.3 : "AAMMJJ_N" -> un seul chiffre de révision (1 à 9)
#define ENTETE_LOG          "DATE;TEMP_AIR;HYGR;PRESSURE;LUMIN;LATITUDE;LONGITUDE"

// VALEURS PAR DÉFAUT DES PARAMÈTRES (modifiables en mode configuration et par CONFIG.TXT)

#define LOG_INTERVAL_DEFAUT   10U    // intervalle entre deux mesures (en unités LOG_INTERVAL_UNITE_MS)
#define LOG_INTERVAL_UNITE_MS 1000U  // TEST : 1 unité = 1 s  |  FINAL : 60000U (1 unité = 1 min)
#define FILE_MAX_SIZE_DEFAUT  512U   // taille max d'un fichier de log (octets) - TEST : 512  |  FINAL : 2048U
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

// MOTIFS LED DES ERREURS (rouge en alternance avec une seconde couleur)

typedef struct
{
    uint8_t r, g, b;        // seconde couleur
    bool    double_duree;   // true : seconde couleur 2x plus longue que le rouge
} motif_erreur_t;

static const motif_erreur_t motifs_erreur[] =
{
    [ERR_GPS]        = {255, 255,   0, false}, // jaune
    [ERR_CAPTEUR]    = {  0, 255,   0, false}, // vert
    [ERR_INCOHERENT] = {  0, 255,   0, true }, // vert, 2x plus long
    [ERR_RTC]        = {  0,   0, 255, false}, // bleu
    [ERR_SD_PLEINE]  = {255, 255, 255, false}, // blanc
    [ERR_SD_ACCES]   = {255, 255, 255, true }  // blanc, 2x plus long
};

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

// PARAMÈTRES
// Tous en int32_t : un seul type pour la table des commandes (pas de pointeur converti vers une autre taille)

typedef struct
{
    int32_t log_interval;   // en unités LOG_INTERVAL_UNITE_MS (min en version finale)
    int32_t file_max_size;  // octets
    int32_t timeout;        // s
    int32_t lumin;
    int32_t lumin_low;
    int32_t lumin_high;
    int32_t temp_air;
    int32_t min_temp_air;
    int32_t max_temp_air;
    int32_t hygr;
    int32_t hygr_mint;
    int32_t hygr_maxt;
    int32_t pressure;
    int32_t pressure_min;
    int32_t pressure_max;
} parametres_t;

// Description d'un paramètre modifiable par la commande NOM=valeur
typedef struct
{
    const char *nom;
    int32_t    *valeur;
    int32_t     min;
    int32_t     max;
} param_config_t;

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

static const parametres_t params_defaut =
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

static parametres_t params; // copié depuis params_defaut au démarrage, puis CONFIG.TXT

// Table des commandes NOM=valeur : nom, variable, valeur min, valeur max
static const param_config_t table_params[] =
{
    {"LOG_INTERVAL",  &params.log_interval,    1,  1440},  // au-delà de 24 h : dépassement du calcul en ms
    {"FILE_MAX_SIZE", &params.file_max_size, 512, 16384},
    {"TIMEOUT",       &params.timeout,         1,   300},
    {"LUMIN",         &params.lumin,           0,     1},
    {"LUMIN_LOW",     &params.lumin_low,       0,  1023},
    {"LUMIN_HIGH",    &params.lumin_high,      0,  1023},
    {"TEMP_AIR",      &params.temp_air,        0,     1},
    {"MIN_TEMP_AIR",  &params.min_temp_air,  -40,    85},  // plage de mesure du BME280
    {"MAX_TEMP_AIR",  &params.max_temp_air,  -40,    85},
    {"HYGR",          &params.hygr,            0,     1},
    {"HYGR_MINT",     &params.hygr_mint,     -40,    85},
    {"HYGR_MAXT",     &params.hygr_maxt,     -40,    85},
    {"PRESSURE",      &params.pressure,        0,     1},
    {"PRESSURE_MIN",  &params.pressure_min,  300,  1100},  // plage de mesure du BME280
    {"PRESSURE_MAX",  &params.pressure_max,  300,  1100}
};

#define NB_PARAMS  (sizeof(table_params) / sizeof(table_params[0]))

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

static const char *const noms_jours[8] = {"", "MON", "TUE", "WED", "THU", "FRI", "SAT", "SUN"};

// PROTOTYPES DES FONCTIONS

// Initialisation et modes
static void    initialiser_systeme(void);           // capteurs, RTC, GPS, carte SD, console, CONFIG.TXT
static void    initialiser_mode_demarrage(void);    // mode au démarrage (bouton rouge maintenu = configuration)
static void    changer_mode(mode_fonctionnement_t nouveau_mode);
static void    gestion_mode(void);                  // appelle la fonction du mode actif
static void    mode_standard_ou_eco(uint8_t facteur);  // acquisition + SD, facteur = 1 (standard) ou FACTEUR_ECO
static void    mode_maintenance(void);              // acquisition + affichage série, sans SD
static void    mode_configuration(void);            // commandes console, délai d'inactivité

// Boutons et LED
static void    appui_long_rouge(void);
static void    appui_long_vert(void);
static void    afficher_couleur_mode(void);
static void    mettre_a_jour_led(void);             // erreur prioritaire en clignotement, sinon couleur du mode

// Acquisition
static uint8_t acquerir_mesures(uint8_t lire_gps);  // capteurs + RTC (+ GPS si lire_gps), retourne 1 si horodatage valide
static void    afficher_mesures(void);              // affichage série des mesures (mode maintenance)
static void    compter_reponse(capteur_t capteur, bool reponse);
static float   moyenne_ajouter(moyenne_t *m, float valeur);
static void    moyenne_vider(moyenne_t *m);
static void    reinitialiser_moyennes(void);

// Erreurs
static void    signaler_erreur(code_erreur_t code); // active le bit de l'erreur dans erreurs_actives
static void    effacer_erreur(code_erreur_t code);  // efface le bit de l'erreur dans erreurs_actives

// Carte SD
static void    enregistrer_sd(void);                // ligne de mesure, changement de date, archivage, écriture
static void    construire_nom_fichier(char *nom, size_t taille, const RTC_DateTime *dt, uint8_t revision);
static void    construire_ligne(char *ligne, size_t taille);
static void    ajouter_champ(char *ligne, size_t taille, bool ok, double valeur, uint8_t decimales);
static uint8_t revision_suivante(const RTC_DateTime *dt);
static SDResultat archiver(const RTC_DateTime *dt);
static void    lire_config_sd(void);                // applique CONFIG.TXT (s'il existe)
static void    appliquer_ligne_config(const char *ligne);
static bool    sauvegarder_config_sd(void);         // réécrit CONFIG.TXT avec les paramètres actuels

// Configuration
static void    traiter_commande(char *commande);    // lecture et exécution d'une commande console
static void    afficher_aide(void);
static void    mettre_en_majuscules(char *texte);
static const char *verifier_coherence(void);
static bool    lire_entier(const char *texte, int32_t *valeur);
static uint8_t jours_dans_mois(int mois, int annee);
static void    commande_clock(const char *arg);
static void    commande_date(const char *arg);
static void    commande_day(const char *arg);
static bool    commande_parametre(const char *nom, const char *texte_valeur, bool verifier_seuils);

// INITIALISATION DU SYSTÈME

static void initialiser_systeme(void)
{
    Global_Init();       // HAL, horloge système, I2C, SPI, UART console
    Console_Init();      // réception des commandes console (USART2 par interruption)
    GroveRGB_Init();     // LED RGB
    GroveButton_Init();  // boutons
    GroveLight_Init();   // capteur de luminosité (ADC)
    GroveBME280_Init();  // capteur BME280 (I2C)
    GroveGPS_Init();     // GPS (UART4 par interruption)
    RTCInterne_Init();   // RTC interne (LSE ou LSI)

    params = params_defaut;   // valeurs par défaut...
    SDLogger_Init();          // carte SD (SPI + FatFs)
    lire_config_sd();         // ...remplacées par celles de CONFIG.TXT s'il existe
}

// MODE STANDARD / ÉCONOMIQUE

static void mode_standard_ou_eco(uint8_t facteur)
{
    static uint32_t derniere_mesure = 0U;
    static uint8_t  compteur_gps = 0U;
    static bool     premiere_mesure = true;

    uint32_t intervalle_ms = (uint32_t)params.log_interval * LOG_INTERVAL_UNITE_MS * facteur;

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

    if (acquerir_mesures(lire_gps)) // horodatage valide : nécessaire pour nommer le fichier
    {
        enregistrer_sd();
    }
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
    char commande[TAILLE_COMMANDE];

    if (Console_LireLigne(commande, sizeof(commande)))
    {
        config_derniere_activite = HAL_GetTick(); // une commande reçue relance le délai d'inactivité
        traiter_commande(commande);
    }

    // Test après la commande : EXIT a pu changer de mode
    if (mode_actuel == MODE_CONFIGURATION && (HAL_GetTick() - config_derniere_activite) >= CONFIG_TIMEOUT_MS)
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

            if (reponse && env.temperature_c >= params.hygr_mint && env.temperature_c <= params.hygr_maxt)
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
        bool reponse = (lumiere != LUMIN_ERREUR) && GroveLight_EstConnecte();

        compter_reponse(CAPT_LUMINOSITE, reponse);

        if (reponse)
        {
            mesures.luminosite    = (uint16_t)(moyenne_ajouter(&moy_luminosite, (float)lumiere) + 0.5f);
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

// AFFICHAGE DES MESURES (mode maintenance)

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

// ENREGISTREMENT SUR CARTE SD

// Nom du fichier de log : AAMMJJ_N.LOG (N = 0 pour le fichier en cours, 1 à 9 pour les archives)
static void construire_nom_fichier(char *nom, size_t taille, const RTC_DateTime *dt, uint8_t revision)
{
    snprintf(nom, taille, "%02u%02u%02u_%u.LOG",
             (unsigned)(dt->year % 100U), dt->month, dt->day, revision);
}

// Ajoute ";valeur" (ou ";NA") à la fin de la ligne
static void ajouter_champ(char *ligne, size_t taille, bool ok, double valeur, uint8_t decimales)
{
    size_t n = strlen(ligne);

    if (ok)
        snprintf(&ligne[n], taille - n, ";%.*f", decimales, valeur);
    else
        snprintf(&ligne[n], taille - n, ";" VALEUR_NA);
}

// Ligne de mesure : JJ/MM/AAAA HH:MM:SS;T;H;P;L;LAT;LON
static void construire_ligne(char *ligne, size_t taille)
{
    snprintf(ligne, taille, "%02u/%02u/%04u %02u:%02u:%02u",
             mesures.horodatage.day, mesures.horodatage.month, mesures.horodatage.year,
             mesures.horodatage.hours, mesures.horodatage.minutes, mesures.horodatage.seconds);

    ajouter_champ(ligne, taille, mesures.temperature_ok, mesures.temperature_c, 1U);
    ajouter_champ(ligne, taille, mesures.humidite_ok,    mesures.humidite_pct,  1U);
    ajouter_champ(ligne, taille, mesures.pression_ok,    mesures.pression_hpa,  1U);
    ajouter_champ(ligne, taille, mesures.luminosite_ok,  mesures.luminosite,    0U);
    ajouter_champ(ligne, taille, mesures.gps_ok,         mesures.latitude,      6U);
    ajouter_champ(ligne, taille, mesures.gps_ok,         mesures.longitude,     6U);
}

// Première révision libre du jour (1 à REVISION_MAX), 0 si toutes sont prises
static uint8_t revision_suivante(const RTC_DateTime *dt)
{
    char nom[16];

    for (uint8_t revision = 1U; revision <= REVISION_MAX; revision++)
    {
        construire_nom_fichier(nom, sizeof(nom), dt, revision);

        if (!SDLogger_Existe(nom))
            return revision;
    }

    return 0U;
}

// Archivage : AAMMJJ_0.LOG renommé en AAMMJJ_N.LOG, l'écriture reprend dans un _0 neuf
static SDResultat archiver(const RTC_DateTime *dt)
{
    uint8_t revision = revision_suivante(dt);

    if (revision == 0U)
    {
        printf("[SD] %u archives deja creees aujourd'hui : le fichier _0 continue de grossir\r\n", REVISION_MAX);
        return SD_OK; // pas de perte de données : on préfère dépasser FILE_MAX_SIZE
    }

    char actuel[16], archive[16];
    construire_nom_fichier(actuel,  sizeof(actuel),  dt, 0U);
    construire_nom_fichier(archive, sizeof(archive), dt, revision);

    SDResultat res = SDLogger_Renommer(actuel, archive);

    if (res == SD_OK)
        printf("[SD] archivage : %s -> %s\r\n", actuel, archive);

    return res;
}

static void enregistrer_sd(void)
{
    char nom[16];
    char ligne[TAILLE_LIGNE];

    construire_nom_fichier(nom, sizeof(nom), &mesures.horodatage, 0U); // nouveau jour = nouveau fichier
    construire_ligne(ligne, sizeof(ligne));

    SDResultat res = SD_OK;
    int32_t taille = SDLogger_TailleFichier(nom); // -1 : fichier absent (ou carte inaccessible)

    // Archivage si la ligne ferait dépasser FILE_MAX_SIZE
    if (taille >= 0 && (uint32_t)taille + strlen(ligne) + 2U > (uint32_t)params.file_max_size)
    {
        res = archiver(&mesures.horodatage);
        taille = SDLogger_TailleFichier(nom);
    }

    // Nouveau fichier : ligne d'en-tête
    if (res == SD_OK && taille < 0)
    {
        res = SDLogger_AjouterLigne(nom, ENTETE_LOG);
    }

    if (res == SD_OK)
    {
        res = SDLogger_AjouterLigne(nom, ligne);
    }

    // Mise à jour des erreurs SD (affichées par la LED)
    if (res == SD_ERR_PLEINE) signaler_erreur(ERR_SD_PLEINE); else effacer_erreur(ERR_SD_PLEINE);
    if (res == SD_ERR_ACCES)  signaler_erreur(ERR_SD_ACCES);  else effacer_erreur(ERR_SD_ACCES);
}

static uint8_t nb_params_lus = 0U; // compteur pour le résumé affiché au démarrage

// CONFIGURATION SUR CARTE SD (CONFIG.TXT : une ligne NOM=valeur par paramètre, # = commentaire)
//
// Lecture : toutes les lignes sont appliquées, puis la cohérence (seuil bas < seuil haut) est vérifiée
// une seule fois à la fin. L'ordre des lignes dans le fichier n'a donc pas d'importance.
// Écriture : CONFIG.TMP est écrit en entier, puis remplace CONFIG.TXT. Une coupure de courant pendant
// l'écriture laisse donc toujours un fichier complet (CONFIG.TXT, ou CONFIG.TMP s'il était déjà renommé).

static void appliquer_ligne_config(const char *ligne)
{
    char copie[SD_TAILLE_LIGNE_MAX];

    strncpy(copie, ligne, sizeof(copie) - 1U);
    copie[sizeof(copie) - 1U] = '\0';
    mettre_en_majuscules(copie);

    if (copie[0] == '\0' || copie[0] == '#')
        return; // ligne vide ou commentaire

    char *egal = strchr(copie, '=');
    if (egal == 0)
    {
        printf("[CONFIG] ligne ignoree : %s\r\n", copie);
        return;
    }

    *egal = '\0';
    if (commande_parametre(copie, egal + 1, false)) // seuils vérifiés après lecture complète du fichier
    {
        nb_params_lus++;
    }
}

static void lire_config_sd(void)
{
    const char *fichier = FICHIER_CONFIG;

    if (!SDLogger_Existe(fichier) && SDLogger_Existe(FICHIER_CONFIG_TMP))
    {
        fichier = FICHIER_CONFIG_TMP; // coupure de courant pendant la dernière sauvegarde
    }

    nb_params_lus = 0U;

    if (!SDLogger_LireLignes(fichier, appliquer_ligne_config))
    {
        printf("[CONFIG] %s absent : valeurs par defaut\r\n", FICHIER_CONFIG);
        return;
    }

    const char *incoherence = verifier_coherence();
    if (incoherence != 0)
    {
        printf("[CONFIG] %s incoherent (%s) : valeurs par defaut\r\n", fichier, incoherence);
        params = params_defaut;
        return;
    }

    printf("[CONFIG] %s charge (%u parametres)\r\n", fichier, nb_params_lus);
}

static bool sauvegarder_config_sd(void)
{
    char ligne[40];

    // 1. Écriture complète dans le fichier temporaire
    SDResultat res = SDLogger_Supprimer(FICHIER_CONFIG_TMP);

    if (res == SD_OK)
    {
        res = SDLogger_AjouterLigne(FICHIER_CONFIG_TMP, "# Configuration WWW (ecrite par le mode configuration)");
    }

    for (size_t i = 0U; i < NB_PARAMS && res == SD_OK; i++)
    {
        snprintf(ligne, sizeof(ligne), "%s=%ld", table_params[i].nom, (long)*table_params[i].valeur);
        res = SDLogger_AjouterLigne(FICHIER_CONFIG_TMP, ligne);
    }

    // 2. Remplacement de l'ancien fichier
    if (res == SD_OK) res = SDLogger_Supprimer(FICHIER_CONFIG);
    if (res == SD_OK) res = SDLogger_Renommer(FICHIER_CONFIG_TMP, FICHIER_CONFIG);

    if (res != SD_OK)
    {
        printf("Attention : sauvegarde dans %s impossible (carte SD ?) : valeur appliquee jusqu'au redemarrage\r\n",
               FICHIER_CONFIG);
        return false;
    }

    return true;
}

// MODE CONFIGURATION : COMMANDES CONSOLE

static void afficher_aide(void)
{
    printf("Commandes : PARAMETRE=valeur, CLOCK=HH:MM:SS, DATE=MM,JJ,AAAA, DAY=MON..SUN, RESET, VERSION, EXIT\r\n");
}

static void mettre_en_majuscules(char *texte)
{
    for (char *c = texte; *c != '\0'; c++)
    {
        if (*c >= 'a' && *c <= 'z') *c = (char)(*c - 'a' + 'A');
    }
}

// Vérifie que chaque seuil bas est strictement inférieur à son seuil haut
static const char *verifier_coherence(void)
{
    if (params.lumin_low    >= params.lumin_high)   return "LUMIN_LOW doit etre < LUMIN_HIGH";
    if (params.min_temp_air >= params.max_temp_air) return "MIN_TEMP_AIR doit etre < MAX_TEMP_AIR";
    if (params.hygr_mint    >= params.hygr_maxt)    return "HYGR_MINT doit etre < HYGR_MAXT";
    if (params.pressure_min >= params.pressure_max) return "PRESSURE_MIN doit etre < PRESSURE_MAX";
    return 0; // cohérent
}

// Conversion stricte d'un entier : refuse "", "12abc", "1.5"
static bool lire_entier(const char *texte, int32_t *valeur)
{
    char *fin;
    long v = strtol(texte, &fin, 10);

    if (fin == texte || *fin != '\0') return false;

    *valeur = (int32_t)v;
    return true;
}

static uint8_t jours_dans_mois(int mois, int annee)
{
    static const uint8_t jours[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    bool bissextile = ((annee % 4 == 0) && (annee % 100 != 0)) || (annee % 400 == 0);

    return (mois == 2 && bissextile) ? 29U : jours[mois - 1];
}

// CLOCK=HH:MM:SS
static void commande_clock(const char *arg)
{
    int h, m, s;
    char reste;
    RTC_DateTime dt;

    if (sscanf(arg, "%d:%d:%d%c", &h, &m, &s, &reste) != 3)
    {
        printf("Erreur : format attendu CLOCK=HH:MM:SS\r\n");
        return;
    }
    if (h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59)
    {
        printf("Erreur : heure hors limites\r\n");
        return;
    }
    if (!RTCInterne_GetDateTime(&dt))
    {
        printf("Erreur : lecture de l'horloge impossible\r\n");
        return;
    }

    dt.hours   = (uint8_t)h;
    dt.minutes = (uint8_t)m;
    dt.seconds = (uint8_t)s;

    if (RTCInterne_SetDateTime(&dt))
        printf("Heure reglee a %02d:%02d:%02d\r\n", h, m, s);
    else
        printf("Erreur : ecriture de l'horloge impossible\r\n");
}

// DATE=MM,JJ,AAAA
static void commande_date(const char *arg)
{
    int mois, jour, annee;
    char reste;
    RTC_DateTime dt;

    if (sscanf(arg, "%d,%d,%d%c", &mois, &jour, &annee, &reste) != 3)
    {
        printf("Erreur : format attendu DATE=MM,JJ,AAAA\r\n");
        return;
    }
    if (annee < 2000 || annee > 2099 || mois < 1 || mois > 12 || jour < 1 || jour > jours_dans_mois(mois, annee))
    {
        printf("Erreur : date invalide\r\n");
        return;
    }
    if (!RTCInterne_GetDateTime(&dt))
    {
        printf("Erreur : lecture de l'horloge impossible\r\n");
        return;
    }

    dt.month = (uint8_t)mois;
    dt.day   = (uint8_t)jour;
    dt.year  = (uint16_t)annee; // année complète : le pilote RTC gère lui-même le stockage sur 2 chiffres

    if (RTCInterne_SetDateTime(&dt) && RTCInterne_GetDateTime(&dt))
        printf("Date reglee au %02d/%02d/%04d (%s)\r\n", jour, mois, annee, noms_jours[dt.dayOfWeek]);
    else
        printf("Erreur : ecriture de l'horloge impossible\r\n");
}

// DAY=MON..SUN : le jour est calculé à partir de la date, la commande vérifie la concordance
static void commande_day(const char *arg)
{
    uint8_t jour_demande = 0U;
    RTC_DateTime dt;

    for (uint8_t i = 1U; i <= 7U; i++)
    {
        if (strcmp(arg, noms_jours[i]) == 0) jour_demande = i;
    }

    if (jour_demande == 0U)
    {
        printf("Erreur : jour attendu MON, TUE, WED, THU, FRI, SAT ou SUN\r\n");
        return;
    }
    if (!RTCInterne_GetDateTime(&dt))
    {
        printf("Erreur : lecture de l'horloge impossible\r\n");
        return;
    }

    if (dt.dayOfWeek == jour_demande)
        printf("Jour confirme : %s\r\n", noms_jours[jour_demande]);
    else
        printf("Attention : la date reglee tombe un %s (jour calcule automatiquement). Corriger avec DATE=\r\n",
               noms_jours[dt.dayOfWeek]);
}

// NOM=valeur (paramètre de la table). verifier_seuils = false pendant la lecture de CONFIG.TXT.
// Retourne true si la valeur a été appliquée.
static bool commande_parametre(const char *nom, const char *texte_valeur, bool verifier_seuils)
{
    const param_config_t *p = 0;

    for (size_t i = 0U; i < NB_PARAMS; i++)
    {
        if (strcmp(nom, table_params[i].nom) == 0) p = &table_params[i];
    }

    if (p == 0)
    {
        printf("Erreur : parametre inconnu (%s)\r\n", nom);
        return false;
    }

    int32_t valeur;
    if (!lire_entier(texte_valeur, &valeur))
    {
        printf("Erreur : %s attend un nombre entier\r\n", nom);
        return false;
    }
    if (valeur < p->min || valeur > p->max)
    {
        printf("Erreur : %s doit etre entre %ld et %ld\r\n", nom, (long)p->min, (long)p->max);
        return false;
    }

    // Application, puis annulation si un seuil bas dépasse son seuil haut
    int32_t ancienne = *p->valeur;
    *p->valeur = valeur;

    const char *incoherence = verifier_seuils ? verifier_coherence() : 0;
    if (incoherence != 0)
    {
        *p->valeur = ancienne;
        printf("Erreur : %s (valeur refusee)\r\n", incoherence);
        return false;
    }

    if (verifier_seuils) // en mode configuration (pas pendant la lecture de CONFIG.TXT)
    {
        printf("%s = %ld\r\n", nom, (long)valeur);
    }
    return true;
}

// Lecture et exécution d'une commande (ligne reçue sur la console)
static void traiter_commande(char *commande)
{
    mettre_en_majuscules(commande); // "log_interval=5" est accepté

    if (strcmp(commande, "VERSION") == 0)
    {
        printf("WWW - version %s, lot %s\r\n", VERSION_PROGRAMME, NUMERO_LOT);
        return;
    }
    if (strcmp(commande, "RESET") == 0)
    {
        params = params_defaut;
        printf("Parametres remis aux valeurs par defaut\r\n");
        (void)sauvegarder_config_sd();
        return;
    }
    if (strcmp(commande, "EXIT") == 0)
    {
        printf("Sortie du mode configuration\r\n");
        changer_mode(MODE_STANDARD);
        return;
    }

    char *egal = strchr(commande, '=');
    if (egal == 0)
    {
        printf("Erreur : commande inconnue (%s)\r\n", commande);
        afficher_aide();
        return;
    }

    *egal = '\0';                 // commande = nom, egal + 1 = valeur
    const char *argument = egal + 1;

    if      (strcmp(commande, "CLOCK") == 0) commande_clock(argument);
    else if (strcmp(commande, "DATE")  == 0) commande_date(argument);
    else if (strcmp(commande, "DAY")   == 0) commande_day(argument);
    else if (commande_parametre(commande, argument, true))
    {
        (void)sauvegarder_config_sd(); // la valeur survit au redémarrage
    }
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

// AFFICHAGE LED : erreur prioritaire en clignotement, sinon couleur du mode

static void mettre_a_jour_led(void)
{
    static int8_t  erreur_affichee = -2;   // -2 : rien d'affiché, -1 : couleur du mode
    static uint8_t phase_affichee  = 0xFFU;
    static mode_fonctionnement_t mode_affiche = MODE_STANDARD;

    // Recherche de l'erreur la plus prioritaire (bit le plus haut)
    int8_t erreur = -1;

    for (int8_t i = (int8_t)ERR_SD_ACCES; i >= 0; i--)
    {
        if (erreurs_actives & (1U << i))
        {
            erreur = i;
            break;
        }
    }

    // Aucune erreur : couleur du mode (renvoyée seulement si elle a changé)
    if (erreur < 0)
    {
        if (erreur_affichee != -1 || mode_affiche != mode_actuel)
        {
            afficher_couleur_mode();
            erreur_affichee = -1;
            mode_affiche = mode_actuel;
        }
        return;
    }

    // Erreur : rouge pendant 1/2 période (ou 1/3 si la seconde couleur dure 2x plus), puis seconde couleur
    const motif_erreur_t *motif = &motifs_erreur[erreur];
    uint32_t duree_rouge_ms = motif->double_duree ? (LED_PERIODE_MS / 3U) : (LED_PERIODE_MS / 2U);
    uint8_t  phase = ((HAL_GetTick() % LED_PERIODE_MS) < duree_rouge_ms) ? 0U : 1U;

    // La LED n'est mise à jour qu'au changement d'erreur ou de phase
    if (erreur != erreur_affichee || phase != phase_affichee)
    {
        if (phase == 0U)
            GroveRGB_SetColor(255, 0, 0);
        else
            GroveRGB_SetColor(motif->r, motif->g, motif->b);

        erreur_affichee = erreur;
        phase_affichee  = phase;
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
            Console_Vider();                          // ignore une ligne tapée avant d'entrer dans ce mode
            afficher_aide();
            config_derniere_activite = HAL_GetTick(); // chronomètre d'inactivité démarré
            break;

        default:
            break;
    }
}

// INITIALISATION DU MODE AU DEMARRAGE

static void initialiser_mode_demarrage(void)
{
    if (GroveButton1_IsPressed()) // bouton rouge maintenu au démarrage
    {
        mode_actuel = MODE_CONFIGURATION;
        Console_Vider();
        config_derniere_activite = HAL_GetTick();
        printf("Demarrage en mode configuration\r\n");
        afficher_aide();
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
    initialiser_systeme();
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
        mettre_a_jour_led();
    }
}