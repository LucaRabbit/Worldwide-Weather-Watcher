#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Accès aux fichiers de la carte SD (FatFs, noms courts 8.3 : "261020_0.LOG", "CONFIG.TXT").
 * Si la carte est retirée puis réinsérée, elle est remontée automatiquement à l'opération suivante.
 */

typedef enum
{
    SD_OK = 0,
    SD_ERR_ACCES,   // carte absente, illisible, non formatée en FAT, ou fichier introuvable
    SD_ERR_PLEINE   // plus de place sur la carte
} SDResultat;

#define SD_TAILLE_LIGNE_MAX  128U  // longueur max d'une ligne lue par SDLogger_LireLignes()

bool       SDLogger_Init(void);      // initialise la carte et monte le système de fichiers
bool       SDLogger_EstPrete(void);  // true si la carte est montée

// Ajoute "ligne\r\n" à la fin du fichier (créé s'il n'existe pas).
// Si la carte est pleine, le fichier est laissé intact (pas de ligne tronquée).
SDResultat SDLogger_AjouterLigne(const char *nom, const char *ligne);

int32_t    SDLogger_TailleFichier(const char *nom);                 // en octets, -1 si absent ou erreur
bool       SDLogger_Existe(const char *nom);
SDResultat SDLogger_Renommer(const char *ancien, const char *nouveau); // échoue si "nouveau" existe déjà

// Appelle traiter() pour chaque ligne du fichier (sans "\r\n"). false si fichier absent ou erreur.
bool       SDLogger_LireLignes(const char *nom, void (*traiter)(const char *ligne));

bool       SDLogger_EspaceLibre(uint32_t *ko_libres);               // espace libre en kio

SDResultat SDLogger_Supprimer(const char *nom);                     // SD_OK aussi si le fichier n'existe pas

#endif