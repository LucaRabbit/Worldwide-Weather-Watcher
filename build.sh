#!/usr/bin/env bash
#
# build.sh - Nettoyage, compilation et téléversement du projet Worldwide Weather Watcher (WWW)
#            sur la carte STM32 Nucleo L476RG, avec PlatformIO.
#
# À lancer depuis la racine du projet :
#   ./build.sh          version démo  (délais courts pour la soutenance)
#   ./build.sh rendu    version rendu (délais du cahier des charges)

set -e   # arrêt à la première erreur

# Choix de la version : MODE_DEMO est transmis au compilateur (-D), sans modifier main.c
if [ "$1" = "rendu" ]; then
    export PLATFORMIO_BUILD_FLAGS="-DMODE_DEMO=0"
else
    export PLATFORMIO_BUILD_FLAGS="-DMODE_DEMO=1"
fi

echo "== Nettoyage"            # supprime les fichiers compilés dans l'autre mode
pio run -t clean

echo "== Compilation"          # affiche à la fin l'occupation de la RAM et de la Flash
pio run

echo "== Televersement"        # televersement par le ST-Link intégré à la Nucleo
pio run -t upload

echo "== Termine"