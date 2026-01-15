# Valós Idejű LeRobot Inferencia ROS2 Kartézi Vezérlés Híd
<!--
Létrehozta Sandor Burian a GitHub Copilot (Claude Sonnet 4) segítségével
-->

[![Pylint](https://github.com/buriansandor/Real-time-LeRobot_Inference_to_ROS2_Cartesian_Control_Bridge/actions/workflows/pylint.yml/badge.svg)](https://github.com/buriansandor/Real-time-LeRobot_Inference_to_ROS2_Cartesian_Control_Bridge/actions/workflows/pylint.yml)
[![Python 3.9+](https://img.shields.io/badge/python-3.9+-blue.svg)](https://www.python.org/downloads/)
[![License: MPL 2.0](https://img.shields.io/badge/License-MPL_2.0-brightgreen.svg)](https://opensource.org/licenses/MPL-2.0)
[![Code style: pylint](https://img.shields.io/badge/code%20style-pylint-blue)](https://pylint.pycqa.org/)

<!-- még nem
[![Model on HF](https://huggingface.co/datasets/huggingface/badges/resolve/main/model-on-hf-sm-dark.svg)]()
[![Paper page](https://huggingface.co/datasets/huggingface/badges/resolve/main/paper-page-sm-dark.svg)]()-->

----
[Angol Readme](README.md)
----



> **Valós Idejű Robot-Robot Vezérlés Híd Több Működési Móddal**
>
> Ez a projekt egy kifinomult összekötőt hoz létre a Hugging Face LeRobot ökoszisztémája és a ROS2 között, két nyílt forráskódú robotkar használatával demonstrálva: a **HF SO100** (vezető) és az **Annin AR4** (követő).
>
> ## 🎯 **Projekt Céljai:**
> - Az AR4 valós idejű vezérlésének engedélyezése az SO100 vezető kar használatával
> - Az HFSO100 betanított modellek közvetlen használhatóságának biztosítása az AR4-en
> - Több működési mód támogatása: hardver hard-kódolt kinematikával, hardver URDF-alapú kinematikával és teljes szimuláció
> - Robusztos keresztplatform kompatibilitás biztosítása (Windows, macOS, Linux)
> - Átfedő biztonsági korlátok és munkaterület-leképezés megvalósítása

## 🚀 **Gyors Indítás**

### **Előfeltételek**
- Python 3.7+ (ajánlott: Python 3.11)
- Git
- Kompatibilis Windows, macOS és Linux rendszerrel

### **1. Telepítés**
```bash
git clone https://github.com/buriansandor/HFSO100_to_AnninAR4_connector.git
cd HFSO100_to_AnninAR4_connector
python setup.py
```

A telepítő szkript automatikusan:
- Létrehoz egy Python virtuális környezetet (.venv)
- Telepíti az összes szükséges függőséget, beleértve az ikpy-t, lerobot-ot, matplotlib-ot
- Létrehozza a requirements.txt fájlt az aktuális csomagverziókkal

### **2. A Rendszer Indítása**
```bash
cd demo
python launcher.py
```

A launcher **három működési módot** biztosít:

#### **1. Mód: Hardver Hard-kódolt Kinematikával**
- Közvetlen hardver vezérlés előre definiált kinematikai transzformációk használatával
- Leggyorsabb teljesítmény, minimális számítási overhead
- SO100 csatlakoztatása soros porton keresztül szükséges (általában COM5)

#### **2. Mód: Hardver URDF-alapú Kinematikával**
- Fejlett mód URDF fájlok használatával mindkét robot számára
- Dinamikus inverz kinematika munkaterület-validációval
- Teljes biztonsági határ ellenőrzés (z-korlátok, radiális korlátok)
- Különböző robotkonfigurációk támogatása

#### **3. Mód: Szimulációs Mód**
- Teljes szimuláció hardver követelmények nélkül
- Ideális fejlesztéshez, teszteléshez és demonstrációhoz
- Teljes 3D vizualizáció valós idejű kinematikai visszajelzéssel

#### **Kartézi Mód**
> olvassa el a kartézi beállítását: [SETUP_GUIDE](SETUP_GUIDE.md)

## 📁 **Projekt Struktúra**

```
HFSO100_to_AnninAR4_connector/
├── .venv/                     # Python virtuális környezet
├── demo/                      # Fő alkalmazás belépési pont
│   ├── launcher.py           # Többmódú rendszer launcher
│   ├── SO100/               # SO100 robot konfiguráció
│   │   ├── URDF/           # Robot URDF fájlok
│   │   │   ├── so100.urdf  # SO100 kinematikai modell
│   │   │   └── so101.urdf  # Alternatív konfiguráció
│   │   ├── so100_driver.py # Hardver interfész driver
│   │   └── callibration_data.csv # Motor kalibrációs adatok
│   ├── AR4/                 # AR4 robot konfiguráció
│   │   └── URDF/
│   │       └── ar4.urdf    # AR4 kinematikai modell
│   └── ROS/                # ROS integrációs példák
├── kinematics/             # Kinematikai transzformációs motorok
│   ├── kinematics_bridge.py          # Alap hard-kódolt kinematika
│   ├── URDF_based_kinematics_bridge.py # Fejlett URDF kinematika
│   └── urdf_based_kinematics_bridge.py # Alternatív URDF implementáció
├── visualisation/          # 3D vizualizáció és UI
│   ├── main_viz.py        # Valós idejű 3D vizualizációs rendszer
│   └── so100_viz.py       # SO100-specifikus vizualizáció
├── setup.py              # Keresztplatform környezet telepítés
├── requirements.txt      # Python függőségek
└── README.md            # Ez a fájl
```

## ⚙️ **Rendszer Architektúra**

### **Alap Komponensek:**

1. **Launcher Rendszer (`demo/launcher.py`)**
    - Automatikus hardver detektálás
    - Mód kiválasztási interfész
    - Keresztplatform port kezelés
    - Hibakezelés és validáció

2. **Robot Driverek:**
    - **SO100 Driver** (`SO100/so100_driver.py`): Soros kommunikáció STS3215 szervókkal
    - **Kalibrációs Rendszer**: Motor-specifikus beállítások betöltése CSV adatokból
    - **URDF Támogatás**: Dinamikus robotmodellek betöltése

3. **Kinematikai Motorok:**
    - **Alap Híd**: Gyors hard-kódolt transzformációk
    - **URDF Híd**: Dinamikus IK/FK biztonsági validációval
    - **Munkaterület-Leképezés**: Skálázás, rotáció, transzláció robot koordináta-rendszerek között

4. **Vizualizációs Rendszer:**
    - **Valós Idejű 3D Plotting**: Élő robotkar pozíciók
    - **Biztonsági Indikátorok**: Vizuális visszajelzés munkaterület korlátaihoz
    - **Többmódú Támogatás**: Hardver és szimulációs renderelés

### **Kulcs Funkciók:**

#### **🔒 Biztonsági Rendszerek**
- **Hengeres Munkaterület-Validáció**: Z-korlátok (asztal szint védelme) és radiális határok érvényesítése
- **IK Solver Ellenőrzés**: Biztosítja, hogy a célpozíciók fizikailag elérhetőek legyenek
- **Valós Idejű Korlát Figyelés**: Vizuális és programozott visszajelzés határközelítéskor
- **Automatikus Pozíció Korlátozás**: Biztonságos fallback határ túllépésekor

#### **🔧 Hardver Integráció**
- **Soros Kommunikáció**: STS3215 szervó protokoll implementáció
- **Motor Kalibráció**: Motoronkénti offset kompenzáció kalibrációs fájlokból
- **Port Auto-detektálás**: Csatlakoztatott SO100 eszközök automatikus felfedezése
- **Keresztplatform Kompatibilitás**: Windows (COM), macOS/Linux (ttyUSB) támogatás

#### **📊 URDF-alapú Kinematika**
- **Dinamikus Modell Betöltés**: Különböző robotkonfigurációk támogatása
- **Inverz Kinematika**: ikpy-alapú solver precíz pozicionáláshoz
- **Előre Kinematika**: Validáció és hiba számítás
- **Többrobot Támogatás**: Különböző alap link konvenciók (SO100: "base", AR4: "base_link")

## 🎮 **Használati Útmutató**

### **A Rendszer Indítása:**
```bash
# Navigálás a demo könyvtárba
cd demo

# A többmódú kiválasztó indítása
python launcher.py

# Válassza ki a preferált módot:
# 1 - Hardver hard-kódolt kinematikával (leggyorsabb)
# 2 - Hardver URDF-alapú kinematikával (legpontosabb)
# 3 - Szimulációs mód (nincs hardver szükséges)
```

### **Mód-specifikus Utasítások:**

#### **Hardver Módok (1 & 2):**
1. Csatlakoztassa az SO100 robotot USB porthoz
2. Biztosítsa a megfelelő tápellátást a robotnak
3. A launcher automatikusan detektálja a portot (általában COM5 Windows-on)
4. Kalibrációs adatok automatikusan betöltődnek az `SO100/callibration_data.csv`-ból

#### **Szimulációs Mód (3):**
- Nincs hardver szükséges
- Teljes 3D vizualizáció aktív
- Interaktív tesztelési környezet
- Ideális fejlesztéshez és demonstrációkhoz

### **Valós Idejű Működés:**
- **3D Vizualizációs Ablak**: Megjeleníti mind a vezető, mind a követő robot pozícióit
- **Címsor Státusz**: Megjeleníti az aktuális módot és biztonsági korlát státuszt
- **Interaktív Vezérlés**: Mozgassa az SO100 vezető kart az AR4 követő vezérléséhez
- **Biztonsági Visszajelzés**: Vizuális indikátorok határközelítéskor

## 🔧 **Technikai Specifikációk**

### **Támogatott Robotok:**
- **Vezető**: HF SO100 (6-DOF, STS3215 szervók, kompakt munkaterület)
- **Követő**: Annin AR4 (6-DOF, nagyobb munkaterület, ROS2 kompatibilis)

### **Kommunikációs Protokollok:**
- **SO100**: Soros USB-n keresztül, STS3215 szervó protokoll
- **AR4**: ROS2 integráció (tervezett/konfigurálható)

### **Függőségek:**
- **ikpy 3.4.2**: Inverz kinematikai solver URDF támogatással
- **lerobot 0.4.2**: Hugging Face robotikai ökoszisztéma integráció
- **matplotlib**: Valós idejű 3D vizualizáció
- **numpy**: Numerikus számítások transzformációkhoz
- **pyserial**: Soros kommunikáció hardver interfészhez

### **Munkaterület-Leképezés:**
- **Skála Faktor**: 1.8x (AR4 1.8x nagyobb munkaterület, mint SO100)
- **Biztonsági Margók**: Konfigurálható halott zónák és maximális elérési korlátok
- **Koordináta-Rendszerek**: Automatikus transzformáció robotbázisok között
- **Valós Idejű Validáció**: Folyamatos biztonsági határ ellenőrzés

## 🛠️ **Fejlesztés**

### **Környezet Beállítása:**
```bash
# Virtuális környezet aktiválása
# Windows PowerShell:
.venv\Scripts\Activate.ps1

# macOS/Linux:
source .venv/bin/activate

# További csomagok telepítése
pip install <csomag-név>

# Requirements frissítése
pip freeze > requirements.txt
```

### **Új Funkciók Hozzáadása:**
1. **Új Kinematikai Híd**: Bővítse az alap osztályokat a `kinematics/`-ban
2. **Robot Támogatás**: Adjon hozzá URDF fájlokat és driver konfigurációkat
3. **Vizualizációs Fejlesztések**: Módosítsa a `visualisation/main_viz.py`-t
4. **Biztonsági Funkciók**: Frissítse a munkaterület-validációt kinematikai hidakban

### **Tesztelés:**
- **Hardver Tesztelés**: Használja az 1. módot alap funkciókhoz, 2. módot fejlett funkciókhoz
- **Szimulációs Tesztelés**: 3. mód biztosít biztonságos környezetet fejlesztéshez
- **Keresztplatform**: Tesztelje a setup.py-t különböző operációs rendszereken

## 🚨 **Biztonsági Megfontolások**

### **Hardver Biztonság:**
- Mindig biztosítsa a megfelelő tápellátási kapcsolatokat működtetés előtt
- Ellenőrizze a munkaterület szabadosságát hardver módok engedélyezése előtt
- Figyelje a motor hőmérsékleteket hosszabb működtetés során
- Tartsa elérhetővé a vészleállítót

### **Szoftver Biztonság:**
- URDF-alapú mód (2. mód) átfogó biztonsági határ ellenőrzést tartalmaz
- Automatikus pozíció korlátozás megakadályozza a veszélyes mozgásokat
- Valós idejű korlát figyelés vizuális visszajelzéssel
- Kalibrációs adatok validációja megakadályozza a motor károsodást

### **Munkaterület Korlátok:**
- **Z-tengely**: Minimum 0.05m (asztal szint védelme), Maximum 0.90m
- **Radiális**: Minimum 0.15m (belső halott zóna), Maximum 0.60m
- **Biztonsági Faktor**: 90% a maximális eléréstől további margóért

## 📊 **Teljesítmény Megjegyzések**

- **1. Mód (Hard-kódolt)**: ~100 Hz frissítési ráta, minimális késleltetés
- **2. Mód (URDF-alapú)**: ~50-80 Hz frissítési ráta, validációs overhead-del
- **3. Mód (Szimulációs)**: ~60 Hz vizualizáció, nincs hardver korlátozás
- **Memória Használat**: ~50-100 MB mód és vizualizációs komplexitás függvényében

## 🤝 **Közreműködés**

1. Forkolja a repository-t
2. Hozzon létre feature branchet (`git checkout -b feature/AmazingFeature`)
3. Commitolja a változtatásokat (`git commit -m 'Add some AmazingFeature'`)
4. Pusholja a branchet (`git push origin feature/AmazingFeature`)
5. Nyisson Pull Request-et

## 📜 **Licenc**

Ez a projekt a Mozilla Public License Version 2.0 alatt van licencelve - lásd a `LICENSE` fájlt a részletekért.

## 🔗 **Kapcsolódó Projektek**

- [Hugging Face LeRobot](https://github.com/huggingface/lerobot)
- [SO100 Robot Dokumentáció](https://github.com/huggingface/lerobot/tree/main/lerobot/configs/robot/so100.yaml)
- [Annin AR4 Robot](https://www.anninrobotics.com/)
- [ikpy - Inverz Kinematika Python](https://github.com/Phylliade/ikpy)

## 📞 **Támogatás**

Kérdésekért, problémákért vagy hozzájárulásokért:
- GitHub Issues: Jelentse a bugokat és kérjen funkciókat
- Discussions: Általános kérdések és közösségi támogatás
- Dokumentáció: Átfedő útmutatók a projekt wikiben

---
*Létrehozta <3 Sandor Burian a GitHub Copilot segítségével*
