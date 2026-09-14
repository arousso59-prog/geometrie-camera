# geometrie-camera

Capteur optique pour appareil de géométrie automobile maison, basé sur ESP32-S3 + caméra OV3660.

## Architecture

Le projet reprend la séparation utilisée dans `equilibreuse-esp32` :

- `geometrie-camera.yaml` : configuration matérielle ESPHome, Wi-Fi, caméra et paramètres de test.
- `components/geometrie_camera_app/` : logique applicative C++.
- Le PC récupérera ensuite les mesures par HTTP/JSON et pourra demander une image de contrôle.

## Étape actuelle — V0

Objectif : valider le module caméra avant tout calcul de géométrie.

1. Démarrage ESP32-S3.
2. Initialisation OV3660.
3. Acquisition d'images fixes.
4. Validation résolution / netteté / stabilité.
5. Préparation des classes C++ qui accueilleront la détection de cible et les calculs angulaires.

## Arborescence

```text
geometrie-camera/
├── geometrie-camera.yaml
├── secrets.yaml.example
└── components/
    └── geometrie_camera_app/
        ├── __init__.py
        ├── geometrie_camera_app.h
        ├── geometrie_camera_app.cpp
        ├── camera_manager.h
        ├── camera_manager.cpp
        ├── target_detector.h
        ├── target_detector.cpp
        ├── geometry_measurement.h
        └── geometry_measurement.cpp
```

## Remarque importante sur le brochage caméra

La carte actuelle est bien configurée comme ESP32-S3 et la caméra comme OV3660. Le brochage parallèle de la caméra dépend toutefois de la référence exacte du PCB ESP32-S3-CAM. Les constantes de broches sont regroupées en haut du YAML afin de pouvoir les corriger sans toucher au reste de l'architecture si nécessaire.

## Étapes suivantes

- valider le brochage réel de la carte reçue ;
- obtenir une première image stable ;
- fixer exposition / gain / balance des blancs pour rendre la mesure reproductible ;
- implémenter la détection de cible ;
- passer aux coordonnées sub-pixel puis aux angles ;
- ajouter la calibration optique.
