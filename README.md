# Simulateur Satellite

> [🇬🇧 English version](docs/README.en.md)

Simulateur satellite temps reel construit sur un STM32F446RE (Nucleo) qui communique avec une station au sol ESP32 via UART. Le STM32 execute FreeRTOS avec 8 taches concurrentes et lit des capteurs reels (IMU, barometre) pour simuler les sous-systemes d'un satellite. Les donnees capteurs sont enregistrees sur carte SD comme une boite noire, un ecran OLED pilote par joystick permet de naviguer entre les vues localement, et l'ESP32 heberge un dashboard WiFi pour le monitoring et le controle a distance.

---

## Sommaire

- [Presentation](#presentation)
- [Materiel](#materiel)
- [Cablage](#cablage)
- [Architecture logicielle](#architecture-logicielle)
- [Modes operationnels](#modes-operationnels)
- [Dashboard Web](#dashboard-web)
- [Mise en route](#mise-en-route)
- [Arborescence du projet](#arborescence-du-projet)

---

## Presentation

L'idee de ce projet est de reproduire, a petite echelle, le type d'architecture que l'on retrouve dans un vrai satellite : plusieurs sous-systemes qui tournent en parallele, chacun avec sa propre cadence d'acquisition, un bus de donnees partage, une gestion d'alarmes, un enregistrement des donnees, et une liaison avec une station au sol.

Le **STM32** gere toute la logique "embarquee" : acquisition capteurs, evaluation des alarmes, affichage OLED, enregistrement sur carte SD et emission de telemetrie. Il execute 7 taches FreeRTOS avec une synchronisation par mutex sur les ressources partagees (bus I2C, bus SPI, structures de donnees capteurs). Un module GPS est prevu dans une mise a jour future.

L'**ESP32** joue le role de station au sol. Il recoit la telemetrie via UART, cree un point d'acces WiFi, et sert un dashboard web monopage via WebSocket pour la visualisation des donnees en temps reel et l'envoi de commandes.

<!-- TODO: ajouter un schema d'architecture haut niveau (ex: STM32 <-> UART <-> ESP32 <-> WiFi <-> Navigateur) -->
![Schema d'architecture](docs/img/architecture.jpg)

---

## Materiel

### Composants

| Composant | Role |
|---|---|
| Nucleo-F446RE (STM32F446RET6) | Simulateur satellite — execute FreeRTOS, lit les capteurs, gere les sous-systemes |
| ESP32 DevKit | Station au sol — point d'acces WiFi, serveur web, relais de commandes |
| MPU-6050 | IMU 6 axes (accelerometre + gyroscope), I2C |
| BMP280 | Capteur barometrique (temperature, pression, altitude), I2C |
| SSD1306 128x64 | Ecran OLED, I2C |
| Joystick analogique | 2 axes, navigation dans les ecrans OLED et acquittement des erreurs |
| Module carte Micro SD | SPI, enregistrement brut des donnees (boite noire) |
| Buzzer (actif) | Retour sonore des alarmes |
| LED | Indicateur de heartbeat (pattern de clignotement selon le mode) |

### Cablage

<!-- TODO: ajouter le schema electrique (Fritzing, KiCad, ou dessin a la main) -->
![Schema electrique](docs/img/schematic.jpg)

**Liaison UART STM32 <-> ESP32 :**

| Pin STM32 | Pin ESP32 | Signal |
|---|---|---|
| PA9 (USART1_TX) | GPIO16 (RX2) | Telemetrie STM32 -> ESP32 |
| PA10 (USART1_RX) | GPIO17 (TX2) | Commandes ESP32 -> STM32 |
| GND | GND | Masse commune |

> Les deux cartes ont des **alimentations separees**. Seule la masse (GND) est partagee.

**Peripheriques STM32 :**

| Peripherique | Pins | Bus |
|---|---|---|
| MPU-6050 | PB8 (SCL), PB9 (SDA) | I2C1 |
| BMP280 | PB8 (SCL), PB9 (SDA) | I2C1 |
| SSD1306 OLED | PB8 (SCL), PB9 (SDA) | I2C1 |
| Carte SD | PA5 (SCK), PA6 (MISO), PA7 (MOSI), PB6 (CS) | SPI1 |
| Joystick X | PA0 | ADC1_CH0 |
| Joystick Y | PA4 | ADC1_CH4 |
| Buzzer | PB5 | Sortie GPIO |
| LED | PA8 | Sortie GPIO |
| UART Debug | PA2 (TX), PA3 (RX) | USART2 |

<!-- TODO: ajouter une photo du cablage reel / breadboard -->
![Photo du cablage](docs/img/setup.jpg)

---

## Architecture logicielle

### STM32 — Taches FreeRTOS

Le firmware tourne sur FreeRTOS (API CMSIS-RTOS v2). Toutes les taches s'executent en parallele et partagent les donnees via des structures protegees par mutex.

| Tache | Priorite | Description | Cadence (Nominal / Science / Safe) |
|---|---|---|---|
| `Task_IMU` | AboveNormal | Lecture accelerometre, gyroscope et temperature du MPU-6050 | 20 Hz / 100 Hz / 5 Hz |
| `Task_BMP` | Normal | Lecture temperature, pression du BMP280, calcul d'altitude | 2 Hz / 5 Hz / 0.5 Hz |
| `Task_Alarm` | AboveNormal | Evaluation des conditions d'alarme (inclinaison, temperature, pression, altitude) et pilotage du buzzer | 2 Hz (desactive en Safe) |
| `Task_Display` | Normal | Rendu des 5 ecrans OLED, navigation joystick et acquittement des erreurs | 10 Hz |
| `Task_Telemetry` | BelowNormal | Envoi de telemetrie CSV sur UART2 (debug) et UART1 (vers ESP32) | 5 Hz / 10 Hz / 1 Hz |
| `Task_ESP32` | Normal | Interrogation des commandes recues de l'ESP32 et traitement | 20 Hz |
| `Task_SDLog` | BelowNormal | Enregistrement des donnees capteurs en paquets binaires de 48 octets sur carte SD | 1 Hz |
| `DefaultTask` | Normal | Clignotement LED heartbeat + buzzer en mode ERROR | Continu |

**Ressources partagees et synchronisation :**

Cinq mutex protegent les acces concurrents :
- `mutexIMU` / `mutexBMP` / `mutexAlarm` — structures de donnees capteurs
- `mutexI2C` — bus I2C1 (partage entre IMU, BMP280 et OLED)
- `mutexSPI` — bus SPI1 (carte SD)

### ESP32 — Station au sol

Le firmware ESP32 est un sketch Arduino unique. Il :
1. Cree un point d'acces WiFi (`SATELLITE_SIM`)
2. Sert un dashboard HTML/CSS/JS embarque sur le port 80
3. Maintient une connexion WebSocket (`/ws`) pour les echanges bidirectionnels en temps reel
4. Retransmet la telemetrie UART du STM32 a tous les clients WebSocket connectes
5. Relaie les commandes de l'interface web vers le STM32

### Protocole de communication

**Telemetrie (STM32 -> ESP32) :** CSV via UART a 115200 bauds.

```
AX,AY,AZ,GX,GY,GZ,T_IMU,T_BMP,P,ALT,JX,JY,ALARMS
```

**Commandes (ESP32 -> STM32) :** Texte brut, termine par un saut de ligne.

```
CMD:MODE:NOMINAL
CMD:MODE:SAFE
CMD:MODE:SCIENCE
CMD:MODE:ERROR_LOW
CMD:MODE:ERROR_HIGH
CMD:ALARM:TILT:0.8
CMD:ALARM:TEMP:50.0
CMD:ALARM:PRES_LOW:950.0
CMD:ALARM:PRES_HIGH:1050.0
CMD:ALARM:ALT_HIGH:2000.0
CMD:ALARM:ALT_LOW:-100.0
CMD:DUMP
```

---

## Modes operationnels

Le simulateur satellite dispose de cinq modes de fonctionnement qui affectent les cadences d'acquisition, le comportement de la LED et la logique du buzzer :

| Mode | LED | Buzzer | Cadence capteurs | Notes |
|---|---|---|---|---|
| **Nominal** | Clignotement 500ms | Actif (sur alarme) | Standard | Mode de fonctionnement par defaut |
| **Safe** | Clignotement lent 2s | Desactive | Reduite | Simulation basse consommation, alarmes ignorees |
| **Science** | Clignotement rapide 100ms | Actif (sur alarme) | Maximale | Acquisition haute frequence |
| **Error Low** | Clignotement tres rapide | Bips courts | Standard | Necessite un acquittement joystick (D-G-D) |
| **Error High** | LED fixe allumee | Continu | Standard | Necessite un acquittement joystick (D-G-D) |

Les modes peuvent etre changes a distance depuis le dashboard web ou declenchees localement. Les modes d'erreur necessitent une sequence physique d'acquittement au joystick (Droite -> Gauche -> Droite) pour revenir en mode Nominal.

### Systeme d'alarmes

Quatre conditions d'alarme sont surveillees, chacune avec des seuils configurables :

| Alarme | Seuil par defaut | Condition |
|---|---|---|
| Inclinaison | > 1.5 g | Accelerometre X ou Y depasse la limite |
| Temperature | > 60 °C | Temperature BMP280 au-dessus de la limite |
| Pression | < 900 ou > 1100 hPa | Pression hors de la plage attendue |
| Altitude | < -200 ou > 3000 m | Altitude hors des limites de securite |

Tous les seuils sont modifiables en temps reel depuis le dashboard web.

---

## Dashboard Web

L'ESP32 sert un dashboard autonome accessible en se connectant au reseau WiFi `SATELLITE_SIM` puis en ouvrant `192.168.4.1` dans un navigateur.

<!-- TODO: ajouter une capture d'ecran ou un GIF du dashboard web en action -->
![Capture du dashboard](docs/img/dashboard.gif)

**Fonctionnalites :**

- **Affichage telemetrie temps reel** — Donnees IMU (accelerometre + gyroscope 6 axes) et BMP280 (temperature, pression, altitude) transmises via WebSocket
- **Indicateurs d'alarme** — Retour visuel avec clignotement anime lorsque les seuils sont depasses
- **Controle de mode a distance** — Basculer entre Nominal, Safe, Science, Error Low et Error High depuis l'interface web
- **Seuils d'alarme configurables** — Modification des parametres critiques en temps reel :
  - Seuil d'inclinaison (g)
  - Limite de temperature (°C)
  - Plage de pression (hPa, basse/haute)
  - Limites d'altitude (m, basse/haute)
- **Gestion du mode erreur** — Lorsqu'un seuil d'alarme est depasse, le satellite peut passer automatiquement en mode Error. Le STM32 affiche alors un ecran d'erreur dedie sur l'OLED, le buzzer emet des bips continus (Error High) ou courts (Error Low), et l'operateur doit effectuer une sequence d'acquittement physique au joystick (Droite -> Gauche -> Droite) pour sortir de l'erreur et revenir en mode Nominal.

<!-- TODO: ajouter un GIF montrant le passage en mode erreur depuis le site et l'acquittement joystick sur l'OLED -->
![Mode erreur](docs/img/error_mode.gif)

- **Data dump** — Demande d'un instantane complet des valeurs capteurs actuelles
- **Journal de commandes** — Retour en temps reel de toutes les commandes envoyees au STM32

### Prerequis

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (pour le firmware STM32)
- [Arduino IDE](https://www.arduino.cc/en/software) avec le support des cartes ESP32
- Bibliotheques Arduino : `ESPAsyncWebServer`, `AsyncTCP`

### Flasher le STM32

1. Ouvrir le dossier `STM32_Simulator/` comme projet STM32CubeIDE (ou importer le fichier `.ioc`).
2. Compiler le projet.
3. Connecter la Nucleo-F446RE en USB et flasher.

### Flasher l'ESP32

1. Ouvrir `ESP32_BaseStation/ESP32_BaseStation.ino` dans Arduino IDE.
2. Installer les bibliotheques requises (`ESPAsyncWebServer` et `AsyncTCP`) via le Library Manager.
3. Selectionner la carte ESP32 et le port COM.
4. Televersement.

### Utilisation

1. Alimenter les deux cartes (USB ou alimentation externe). S'assurer que **la masse (GND) est partagee** entre elles.
2. Connecter les lignes UART (PA9 -> GPIO16, PA10 <- GPIO17).
3. Sur un telephone ou un PC, se connecter au reseau WiFi `SATELLITE_SIM` (mot de passe : `sat12345`).
4. Ouvrir un navigateur et aller sur `http://192.168.4.1`.
5. Les donnees de telemetrie s'affichent en temps reel et les commandes peuvent etre envoyees.

<!-- TODO: ajouter un GIF ou une video courte montrant le workflow complet (boot -> dashboard -> changement de mode) -->
![GIF de demo](docs/img/normal_mode.gif)

### Navigation OLED

Le joystick permet de naviguer entre cinq ecrans sur le SSD1306 :
1. **IMU** — Valeurs de l'accelerometre et temperature de l'IMU
2. **BMP280** — Temperature, pression, altitude
3. **Alarmes** — Etat actuel des alarmes
4. **Mode** — Mode actif, etat du buzzer
5. **Statut** — Etat d'initialisation des capteurs, valeurs brutes du joystick

---

## Arborescence du projet

```
.
├── ESP32_BaseStation/
│   └── ESP32_BaseStation.ino      # Firmware station au sol ESP32
├── STM32_Simulator/
│   ├── Core/
│   │   ├── Inc/                   # Headers (main.h, ssd1306.h, FreeRTOSConfig.h, ...)
│   │   ├── Src/                   # Sources (main.c, ssd1306.c, freertos.c, ...)
│   │   └── Startup/               # Assembleur de demarrage
│   ├── Drivers/                   # Drivers HAL & CMSIS
│   ├── Middlewares/               # Noyau FreeRTOS
│   ├── Satelite.ioc               # Fichier projet STM32CubeMX
│   ├── STM32F446RETX_FLASH.ld    # Script de link (flash)
│   └── STM32F446RETX_RAM.ld      # Script de link (RAM)
└── README.md
```
### IA

Certaines images d'illustration (schemas, diagrammes) ont ete generees avec **Gemini**. Les photos du montage reel sont authentiques. **Claude** a ete utilise comme outil d'apprentissage et d'assistance pour certains elements du code.
---

Merci d'avoir pris le temps de regarder ce projet. Si vous avez des questions, des suggestions ou simplement envie d'en discuter, n'hesitez pas a ouvrir une issue ou a me contacter. J'espere que ce projet pourra vous inspirer ou vous etre utile.



