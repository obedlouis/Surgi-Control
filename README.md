#                                                       🏥 Surgi-Control
                                              Interface Médicale Sans Contact sur STM32

#                                                           Description
Surgi-Control est un système embarqué développé sur la carte B-L475E-IOT01A (STM32L475) permettant de contrôler un ordinateur sans contact, spécialement conçu pour les environnements médicaux stériles (bloc opératoire).

#                                                           Objectif : 
réduire les risques de contamination tout en conservant une interaction fluide avec les systèmes médicaux informatiques.

* **Le système remplace clavier et souris grâce à :**
* Reconnaissance gestuelle (capteur ToF)
* Analyse audio temps réel
* Émulation Clavier USB HID

#                                                          Fonctionnalités
**Navigation Gestuelle**
Capteur de distance VL53L0X connecté en I2C.
| Geste détecté                   | Action PC       |
| ------------------------------- | --------------- |
| Main stable à gauche            | Page précédente |
| Main stable à droite            | Page suivante   |
| Mouvement rapide vers l’avant   | Zoom avant      |
| Mouvement rapide vers l’arrière | Zoom arrière    |

**Commandes Audio (Claps)**
Microphone MEMS numérique (MP34DT01) via DFSDM.
Détection de séquences rythmiques en temps réel :
| Nombre de claps | Action        |
| --------------- | ------------- |
| 1 clap          | Scroll bas    |
| 2 claps         | Scroll haut   |
| 3 claps         | Onglet gauche |
| 4 claps         | Onglet droit  |

Traitement audio via DMA + DFSDM
Architecture non bloquante (CPU optimisé)

#                                                           Interface PC
Émulation USB HID Keyboard
Reconnu nativement par : Windows (Microsoft Edge)

#                                                       Architecture Technique
**Matériels**
Carte : B-L475E-IOT01A
Capteur distance : VL53L0X
Microphone : MP34DT01 (MEMS numérique)

**Périphériques STM32 Utilisés**
DMA → Transfert mémoire haute performance
DFSDM → Acquisition audio PDM
I2C → Communication capteur ToF
USB Device (HID) → Émulation clavier
Timers & Interruptions → Gestion temps réel

#                                                       Architecture Fonctionnelle
**Gestes (ToF) ──► Analyse Distance ──► Mapping Commande ──► USB HID
Audio (MEMS) ──► DFSDM + DMA ──► Détection Claps ──► USB HID**

## Auteur
Développé dans le cadre du projet d'ingénierie (Cycle Ingénieur Instrumentation) par **Hiba SEBBAI et Obed LOUIS**
Sup Galilée – Institut Galilée
