# 📖 SO-100 Robotkar Beüzemelési és Kalibrációs Útmutató

> Ez a dokumentum leírja, hogyan kell egy frissen összeszerelt (vagy Hugging Face LeRobot állapotú) SO-100 robotkart felkészíteni a precíziós Cartesian (XYZ) vezérlésre.
## ⚠️ Miért van erre szükség?
A gyári (vagy teleoperációs) beállítások "Master-Slave" másolásra készültek, ezért:
1. Szűk szoftveres limitekkel (Safety Limits) rendelkeznek, amik megakadályozzák a matematikai alapállás (Gyertya) elérését.
2. A 0 pontjuk nem a függőleges, hanem egy kényelmi pozíció (Raptor).

Az Inverz Kinematikához (IK) **korlátlan mozgástartomány** és **pontos matematikai nulla (Gyertya)** szükséges.

## 🛠️ 1. Lépés: Szoftveres Limitek Törlése ("Unlock")
*Cél: A "Gyerekzár" levétele, hogy a robot elérhesse a fizikai végállásokat.*
1. Csatlakoztasd a robotot USB-n (és a 12V tápot).
2. Futtasd a limit-törlő szkriptet (amit korábban írtunk):
    ```Bash
    python unlock_motor_limits.py
    ```

   *(Ha félsz, használd a `backup_and_unlock.py`-t a gyári értékek mentéséhez).*

3. Ellenőrzés: A konzolon látnod kell: Min=0, Max=4095.

## 📏 2. Lépés: A "Szent Gyertya" Kalibráció
*Cél: Megtanítani a robotnak, hogy hol van a valódi, függőleges nulla pont.*

1. Futtasd a manuális kalibrálót:
   ```Bash
    python calibrate_manual_candle.py
   ```

2. A szkript kikapcsolja a motorokat (Torque OFF).
3. KÉZZEL állítsd be a robotot a tökéletes Függőleges (Gyertya) pozícióba:
   - Minden kar egyenesen felfelé mutasson.
   - A csukló egyenes.
   - A derék (1-es motor) pontosan középen álljon.
   - *(Tipp: Használj vízmértéket vagy derékszöget, ha tudsz, de szemmértékkel is jónak kell lennie).*
4. Nyomj **ENTER**-t.
5. A szkript elmenti az értékeket a `follower_calibration.csv` fájlba.
   
   *`Példa értékek: [2059, 2019, 1042, ...]`*

## 🧪 3. Lépés: Tesztelés (Simple Move Test)
*Cél: Ellenőrizni, hogy a robot magától is megtalálja-e a pozíciókat.*

1. Nyisd meg a simple_move_test.py-t.
2. FONTOS: Győződj meg róla, hogy a driver inicializálásánál a calibration_file paraméter a most létrehozott CSV-re mutat (vagy ideiglenesen írd be az értékeket a kódba a [FIX] részhez, ha fájlkezelési gond van).
3. Futtasd a tesztet:
   ```Bash
    python simple_move_test.py
   ```
4. Gyertya Teszt (Gomb: 4):
    - A robotnak lassan, de határozottan fel kell emelkednie függőlegesbe.
    - Ha "Raptorba" megy helyette -> Rossz a kalibrációs fájl/érték! (Lásd 2. lépés).
    - Ha meg se mozdul -> Nincs bekapcsolva a Torque (Nyomaték)! (A szkriptnek tartalmaznia kell a `0x28`-as regiszter írását).
5. Raptor Teszt (Gomb: 6):

        A robotnak össze kell csukódnia a biztonsági parkoló állásba.

## 🚑 Hibaelhárítás (Troubleshooting)
- A robot nem mozdul, csak "zizeg":

        Valószínűleg a sebesség túl nagy, vagy a mechanika szorul. Növeld a move_time értékét (pl. 5000 ms).

- A robot "meghalt" (LED villog, nem válaszol):

        Túláram védelem aktiválódott.

        Megoldás: Húzd ki a 12V-ot ÉS az USB-t is 10 másodpercre.

- Raptorba megy Gyertya helyett:

        A szoftver a régi (Hugging Face / Leader másolt) kalibrációs adatokat használja. Kényszerítsd rá az új, mért értékeket.