#!/usr/bin/env bash
#
# build.sh - Nettoyage, compilation et téléversement du projet Worldwide Weather Watcher (WWW)
#            sur la carte STM32 Nucleo L476RG, avec PlatformIO (pio).
#
# Utilisation (depuis la racine du projet, là où se trouve platformio.ini) :
#   ./build.sh          version démo  (MODE_DEMO=1 : délais courts pour la soutenance)
#   ./build.sh rendu    version rendu (MODE_DEMO=0 : délais du cahier des charges)

set -euo pipefail   # arrêt à la première erreur

ENV="nucleo_l476rg"   # environnement défini dans platformio.ini ([env:nucleo_l476rg])

# MODE DÉMO OU RENDU

case "${1:-demo}" in
    demo)  MODE_DEMO=1 ;;
    rendu) MODE_DEMO=0 ;;
    *)     echo "Utilisation : ./build.sh [demo|rendu]" >&2; exit 1 ;;
esac

# VÉRIFICATIONS

# Le script doit être lancé depuis la racine du projet
if [ ! -f platformio.ini ]; then
    echo "ERREUR : platformio.ini introuvable, lancer le script depuis la racine du projet" >&2
    exit 1
fi

# pio est dans le PATH si PlatformIO Core est installé ; sinon, on prend celui de l'extension VS Code
if command -v pio > /dev/null 2>&1; then
    PIO="pio"
elif [ -x "$HOME/.platformio/penv/bin/pio" ]; then
    PIO="$HOME/.platformio/penv/bin/pio"
else
    echo "ERREUR : pio introuvable (installer PlatformIO Core ou l'extension PlatformIO de VS Code)" >&2
    exit 1
fi

# MODE_DEMO transmis au compilateur (-D), sans modifier main.c
# (main.c doit contenir "#ifndef MODE_DEMO / #define MODE_DEMO 1 / #endif")
export PLATFORMIO_BUILD_FLAGS="-DMODE_DEMO=${MODE_DEMO}"
echo "== Version ${1:-demo} (MODE_DEMO=${MODE_DEMO})"

# 1. NETTOYAGE : supprime les fichiers compilés, tout est recompilé avec le bon MODE_DEMO
echo "== Nettoyage"
"$PIO" run -e "$ENV" -t clean

# 2. COMPILATION : affiche à la fin l'occupation de la RAM et de la Flash
echo "== Compilation"
"$PIO" run -e "$ENV"

# 3. TÉLÉVERSEMENT sur la carte, par le ST-Link intégré à la Nucleo
echo "== Televersement"
"$PIO" run -e "$ENV" -t upload

echo "== Termine"