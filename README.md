# Projet RTOS STM32

Ce projet embarqué utilise **FreeRTOS** sur une carte **STM32** (probablement STM32F4) pour effectuer des tâches concurrentes avec gestion de files (`Queue`) et sémaphores (`Semaphore`). Le microcontrôleur collecte des données depuis des capteurs via **ADC**, les traite, et les envoie via **USART2**.

## Fonctions principales

- Lecture de température via capteur **DS1621** (I2C)
- Lecture de plusieurs canaux **ADC**
- Envoi des données via **USART2**
- Gestion watchdog
- Utilisation de **FreeRTOS** avec 6 tâches :
  - `vTask0` : Activation watchdog
  - `vTask1` : Envoi des données
  - `vTask2` : Traitement des données
  - `vTask3-4-5` : Lecture des canaux ADC

## Matériel requis

- STM32 (ex: STM32F407VG Discovery)
- DS1621 (capteur température)
- Interface série (USART)
- FreeRTOS

## Dépendances

- STM32CubeMX (pour configuration des périphériques)
- FreeRTOS
- Keil uVision / STM32CubeIDE / autre IDE compatible ARM
- HAL drivers

## Compilation

1. Ouvrir le projet dans l'IDE de ton choix (Keil, STM32CubeIDE…)
2. Compiler et flasher le programme sur la carte
3. Monitorer via USART2 (115200 bauds par exemple)

## Organisation des fichiers

- `main.c` : point d'entrée principal
- `projet.h` : déclarations des fonctions/structures
- `FreeRTOS/` : noyau temps réel
- `Drivers/` : pilotes STM32 HAL/I2C/ADC/USART

## Auteur

Mohammed Zribi – 
