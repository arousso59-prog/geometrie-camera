# geometrie-camera

Capteur optique pour appareil de géométrie automobile maison, basé sur ESP32-S3 + caméra OV5640.

## Matériel validé

- carte : GOOUUU ESP32-S3-CAM V1.5, ESP32-S3 N16R8 ;
- capteur confirmé par PID : OV5640 (`0x5640`) ;
- PSRAM : 8 Mo octal ;
- résolution maximale exposée par ESPHome 2026.7.3 : 2560×1920 ;
- XCLK retenu actuellement : 8 MHz.

Brochage caméra :

```text
SIOD  GPIO4
SIOC  GPIO5
VSYNC GPIO6
HREF  GPIO7
XCLK  GPIO15
Y2    GPIO11
Y3    GPIO9
Y4    GPIO8
Y5    GPIO10
Y6    GPIO12
Y7    GPIO18
Y8    GPIO17
Y9    GPIO16
PCLK  GPIO13
```

## Voie image retenue

```text
OV5640 JPEG
   ↓
frame fraîche
   ↓
décodage grayscale 8 bits
   ↓
JpegArtifactCorrector V3
   ↓
buffer grayscale corrigé
   ↓
TargetDetectionService
   ↓
TargetDetector V5.4
   ├── TargetCandidateFinder V5.2
   ├── TargetCornerRefiner V5.4
   └── TargetCodeDecoder V5.4
   ↓
TargetObservation
   ↓
distance / orientation / géométrie
```

Le correcteur V3 supprime la grande majorité des artefacts verts/noirs sans appliquer de flou global.

La V5 sépare désormais trois problèmes :

1. **retrouver rapidement la petite cible dans toute l'image** ;
2. **raffiner ses quatre coins sur l'image pleine résolution** ;
3. **lire le code 7×7 dans le quadrilatère obtenu**.

`TargetCandidateFinder` réduit l'image à environ 320 px maximum, applique un seuillage local puis recherche des composantes sombres quasi carrées. Les 8 meilleurs candidats sont conservés dans un buffer persistant afin de limiter le coût CPU et la pile du handler HTTP.

`TargetCornerRefiner` reprend chaque candidat dans l'image pleine résolution et déplace localement ses quatre coins en recherchant les transitions attendues entre le fond clair et le cadre noir. Si ce raffinement n'est pas suffisamment cohérent, le candidat brut reste utilisé.

`TargetCodeDecoder` teste le candidat brut et, lorsqu'il existe, le candidat raffiné. Depuis la V5.4, le motif 7×7 est projeté avec une homographie projective plutôt qu'une simple interpolation bilinéaire, afin de mieux supporter une cible vue en biais. Le score final est accepté à partir de `0.82` après les gardes indépendantes de contraste, cadre noir et fond extérieur.

Sur les essais réels en 1600×1200 avant ajout du raffinement, la localisation/détection complète était de l'ordre de 0,16 s. Le principal goulot de performance reste le filtre JPEG/correction d'artefacts, autour de 4 s au total sur les mesures actuelles.

## Architecture

Les responsabilités détaillées et les règles de développement sont dans [`ARCHITECTURE.md`](ARCHITECTURE.md).

## API actuelle

```text
GET /api/wsdl
GET /api/runtime/status

GET /api/camera/settings
GET /api/camera/settings/set?<parametres>

GET /diagnostic-jpeg/capture?resolution=<optionnel>
GET /diagnostic-jpeg/status
GET /diagnostic-jpeg/image.jpg
GET /diagnostic-jpeg/filter
GET /diagnostic-jpeg/filter-status
GET /diagnostic-jpeg/filtered.bmp

GET /target/detect
GET /target/status
GET /target/preview.bmp
```

Les routes de cible restent inchangées avec la V5.4 :

- `/target/detect` traite la dernière image déjà filtrée ;
- `/target/status` relit le dernier résultat ;
- `/target/preview.bmp` affiche une miniature annotée du meilleur résultat/candidat.

`/api/wsdl` est la référence du contrat HTTP compilé.

## Étape actuelle

Valider la V5.4 sur la cible réelle :

1. capture JPEG en 1600×1200 ;
2. filtre V3 ;
3. `/target/detect` ;
4. tester d'abord la cible presque droite pour vérifier qu'il n'y a pas de régression ;
5. incliner progressivement la cible et surveiller `target_found`, `quality` et les logs `coarse/refined` ;
6. tester ensuite 800×600 et 1600×1200 à distance comparable ;
7. retirer complètement la cible et vérifier `target_found=false` ;
8. une fois cette robustesse confirmée, passer à la distance puis à l'orientation fine.

Après validation V5.4, la prochaine optimisation de performance doit porter sur `JpegFilteredDiagnostic` / `JpegArtifactCorrector`. La future voie rapide utilisera ensuite une ROI autour de la dernière cible connue et reviendra à la recherche globale en cas de perte.
