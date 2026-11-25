# RoboDraw

RoboDraw rysuje plik SVG pisakiem zamocowanym na robocie Hanwha HCR-3A w RoboDK. Program sam układa kreski w kolejności, podnosi pisak nad papierem i opuszcza go tylko na linii rysunku.

Pracuje w symulatorze albo na prawdziwym ramieniu. Kartkę i końcówkę pisaka ustawia się raz, zapisuje do pliku JSON i podaje przy każdym rysunku. Wtedy rysunek startuje w nauczonym rogu, idzie wzdłuż nauczonych krawędzi i mieści się w prostokącie kartki.

Polecenia uruchamiaj w katalogu `robo-draw`, na tym samym komputerze, na którym jest otwarty RoboDK. Domyślnie program łączy się z `localhost`, port `20500`.

## Czego potrzeba

- Windows z zainstalowanym RoboDK (sprawdzone na 6.2).
- Python 3.12 i [uv](https://docs.astral.sh/uv/).
- Otwarta stacja z robotem o nazwie `Hanwha HCR-3A`.
- Plik SVG. Ścieżki, linie, łuki, okręgi i wielokąty. Program czyta współrzędne ścieżek, a nie kolor ani grubość kreski.

```bash
cd robo-draw
uv sync
```

## Stacja w RoboDK

Przed pierwszym uruchomieniem:

1. Otwórz RoboDK i stację z modelem HCR-3A.
2. Zostaw RoboDK w trybie symulacji, dopóki nie sprawdzisz rysunku na ekranie.
3. Nazwa robota w drzewie stacji ma być `Hanwha HCR-3A`. Inną nazwę podasz przez `--robot-name`.

Układ `Drawing Frame` i narzędzie `Drawing Tool` utworzą się same przy `setup`, jeśli ich nie ma. Końcówka narzędzia ma leżeć na czubku pisaka. `setup` zapisuje tę końcówkę do JSON-a.

Pisak w symulatorze przesuwasz panelem jog robota, a nie przesuwaniem bazy. Baza może stać w zerze stacji. Program czyta pozycję czubka, nie pozycję podstawy.

## Szybki przykład na symulatorze

Trzy polecenia: naucz kartkę, wstaw ją do stacji jeszcze raz gdy otworzysz projekt od nowa, narysuj przykładowego dudka.

```bash
uv run python -m robo_draw.main setup --output sheet.json
```

Program prosi o trzy dojazdy. Po każdym ustawieniu ramienia naciśnij Enter.

```text
1/3  Róg kartki, początek XY. Przesuń robota i naciśnij Enter.
2/3  Koniec dolnej krawędzi, to jest szerokość kartki. Przesuń robota i naciśnij Enter.
3/3  Koniec lewej krawędzi, to jest wysokość kartki. Przesuń robota i naciśnij Enter.
```

Potem:

```bash
uv run python -m robo_draw.main draw --svg-path examples/dudek.svg --setup sheet.json
```

W logu pojawia się linia w tym stylu:

```text
Kartka 210.0 x 297.0 mm, rysunek 190.0 x 250.0, skala 1.0842.
```

To znaczy, że cały rysunek zmieścił się w nauczonej kartce. Skala jest jedna dla obu osi, więc proporcje zostają.

Sam rysunek, bez pliku kartki, jedzie w układzie `Drawing Frame` już stojącym w stacji. Jednostka SVG razy `--scale` to milimetr. Bez `--scale` skala wynosi 1.

```bash
uv run python -m robo_draw.main draw --svg-path examples/dudek.svg --scale 0.5
```

## Uczenie kartki

`setup` zawsze łączy się w symulacji. Trzy punkty wyznaczają prostokąt:

| Krok | Gdzie postawić czubek pisaka | Co z tego wynika |
| --- | --- | --- |
| 1 | Róg, od którego ma się zaczynać rysunek | Początek układu, `X = 0`, `Y = 0` |
| 2 | Drugi koniec dolnej krawędzi | Oś X i szerokość kartki |
| 3 | Drugi koniec lewej krawędzi | Oś Y i wysokość kartki |

Między punktami ramię musi naprawdę zmienić pozycję. Dwa punkty bliżej niż 1 mm program odrzuca i prosi o ponowny dojazd. Trzy punkty na jednej linii też odrzuca, bo nie da się z nich zbudować płaszczyzny.

Oś Z układu kartki jest skierowana w górę stacji. Dodatnie Z to powietrze nad papierem. Podnoszenie pisaka idzie w tę stronę.

Wysokość podnoszenia zapisuje się razem z kartką. Domyślnie 10 mm.

```bash
uv run python -m robo_draw.main setup --output a4.json --approach-dist 15
```

Inne nazwy obiektów w stacji:

```bash
uv run python -m robo_draw.main setup \
  --output kartka-biurko.json \
  --robot-name "Hanwha HCR-3A" \
  --frame-item-name "Drawing Frame" \
  --tool-item-name "Drawing Tool"
```

Gdy stacja jest pusta albo ktoś przesunął układ ręcznie, ten sam plik wstawisz bez uczenia od nowa:

```bash
uv run python -m robo_draw.main setup --output sheet.json --apply
```

`--apply` stawia `Drawing Frame` i końcówkę pisaka tak, jak w pliku. Nie pyta o trzy punkty.

Po udanym uczeniu w konsoli jest rozmiar, na przykład `Kartka 210.0 x 297.0 mm`, i ścieżka pliku.

## Plik sheet.json

Fragment prawdziwego zapisu wygląda tak. Liczby zależą od stacji.

```json
{
  "version": 1,
  "robot_name": "Hanwha HCR-3A",
  "frame_item_name": "Drawing Frame",
  "tool_item_name": "Drawing Tool",
  "approach_dist": 10,
  "frame_pose": [
    [1.0, 0.0, 0.0, 320.0],
    [0.0, 1.0, 0.0, -180.0],
    [0.0, 0.0, 1.0, 150.0],
    [0.0, 0.0, 0.0, 1.0]
  ],
  "frame_xyzrpw": [320.0, -180.0, 150.0, 0.0, 0.0, 0.0],
  "tool_pose_xyzrpw": [0.0, 0.0, 120.0, 0.0, 0.0, 0.0],
  "points_mm": {
    "origin": [320.0, -180.0, 150.0],
    "x": [530.0, -180.0, 150.0],
    "y": [320.0, 117.0, 150.0]
  },
  "sheet_mm": { "width": 210.0, "height": 297.0 },
  "y_sign": 1.0
}
```

- `frame_pose` to macierz układu kartki w stacji. Ostatnia kolumna to róg w milimetrach.
- `frame_xyzrpw` i `tool_pose_xyzrpw` to ta sama poza w milimetrach i stopniach, tak jak w RoboDK.
- `points_mm` to trzy nauczone czubki pisaka.
- `sheet_mm` to szerokość i wysokość prostokąta w milimetrach.
- `y_sign` jest `1`, gdy kartka leży po dodatniej osi Y układu. Jest `-1`, gdy program musiał obrócić oś Y, żeby Z nadal wskazywało w górę. Rysunek i tak ląduje na nauczonej stronie.
- `approach_dist` to wysokość lotu nad papierem. Przy rysowaniu z `--setup` brana jest wartość z pliku, a nie z opcji wiersza poleceń.

Starszy plik bez `sheet_mm` też da się wczytać, jeśli ma `points_mm`. Rozmiar kartki liczy się wtedy z tych trzech punktów. Ma to sens tylko wtedy, gdy drugi i trzeci punkt były na końcach krawędzi.

## Rysowanie

```bash
uv run python -m robo_draw.main draw --help
```

Najczęstsze wywołanie:

```bash
uv run python -m robo_draw.main draw \
  --svg-path examples/dudek.svg \
  --setup sheet.json
```

Co się wtedy dzieje:

1. Program łączy się z RoboDK i przełącza go na symulację.
2. Wstawia układ i pisak z JSON-a.
3. Mierzy prostokąt całego SVG.
4. Przesuwa rysunek tak, żeby jego lewy dolny róg w danych (najmniejsze X i najmniejsze Y ścieżek) spadł na róg kartki, z marginesem.
5. Dobiera skalę. Bez `--scale` rysunek wypełnia kartkę w tym wymiarze, który pierwszy dochodzi do krawędzi.
6. Planuje ruchy i rysuje.

Margines wynosi 2 mm z każdej strony. Na kartce mniejszej niż kilka centymetrów margines schodzi do 5% krótszego boku, żeby został jakiś obszar na kreskę.

`--scale` przy podanym `--setup` nie wychodzi poza kartkę. Wartość mniejsza od automatycznej rysuje mniejszy obraz, w tym samym rogu. Wartość większa jest obcinana do skali, która jeszcze się mieści.

```bash
# mniejszy rysunek w rogu tej samej kartki
uv run python -m robo_draw.main draw --svg-path examples/dudek.svg --setup sheet.json --scale 0.4
```

Bez `--setup` skala 1 oznacza, że liczba w pliku SVG jest milimetrem w `Drawing Frame`. Plik `examples/dudek.svg` ma `viewBox` `0 0 210 297`, a współrzędne ścieżek są w tym zakresie, więc przy skali 1 zajmuje około kartki A4, o ile układ w stacji jest już ustawiony na papier.

Rozdzielczość próbkowania łuków i krzywych to `--resolution`, w milimetrach na punkt. Domyślnie 1 mm.

```bash
uv run python -m robo_draw.main draw \
  --svg-path rysunek.svg \
  --setup sheet.json \
  --resolution 0.5
```

Opcje, których używa się rzadziej:

| Opcja | Domyślnie | Znaczenie |
| --- | --- | --- |
| `--svg-path` | wymagane | Plik SVG |
| `--setup` | brak | JSON kartki. Włącza dopasowanie do prostokąta |
| `--scale` | 1 bez setupu, auto z setupem | Mnożnik. Z setupem nie przekracza kartki |
| `--resolution` | 1 | Gęstość próbkowania, mm |
| `--approach-dist` | 10 | Lot nad papierem, mm. Przy `--setup` wygrywa wartość z pliku |
| `--robot-name` | Hanwha HCR-3A | Nazwa robota w stacji |
| `--frame-item-name` | Drawing Frame | Układ rysunku, gdy nie ma `--setup` |
| `--tool-item-name` | Drawing Tool | Narzędzie, gdy nie ma `--setup` |
| `--run-on-robot` | wyłączone | Po próbie na symulatorze: rusz prawdziwym ramieniem |
| `--force-robot` | wyłączone | Pozwala jechać na ramię, gdy RoboDK nie jest w symulacji |
| `--use-visual-simulation` | wyłączone | Dokleja kopię obiektu `Pixel` do `Drawing Board` wzdłuż kreski |

## Jak jedzie ramię

Na czas rysunku aktywny układ to kartka, a aktywna końcówka to pisak.

1. Ramię zjeżdża do pozycji domowej stawów `[0, -70, -95, -104, 90, -20]` stopni.
2. Nad kartką leci po prostej na wysokości `approach_dist`, szybko, bez zaokrąglania narożników.
3. Schodzi pionowo na papier (`Z = 0` układu kartki).
4. Rysuje odcinki `MoveL` i łuki `MoveC`. Na papierze prędkość jest mniejsza, a narożnik może ściąć o 1 mm, żeby nie stawał na każdej kresce.
5. Przed przelotem do kolejnej kreski najpierw podnosi pisak w tym samym XY, potem leci w bok, potem schodzi.
6. Na końcu wraca do pozycji domowej.

Przelot w bok jest prostą na stałej wysokości. Program nie używa do tego ruchu w stawach, bo taki łuk potrafi zejść poniżej papieru.

Bliskie końce kresek, do 3 mm, są łączone bez podnoszenia pisaka. Siatka trójkątów, taka jak w `examples/dudek.svg`, gubi krawędzie wewnętrzne i zostawia obrys. Okrąg rysowany jest dwoma łukami.

## Przykład: dudek na A4

`examples/dudek.svg` to obrys siatki na stronie o polu 210 na 297 jednostek. Jednostki w ścieżkach są traktowane jak milimetry, zanim dojdzie skala.

Symulacja z dopasowaniem do kartki, którą właśnie nauczyłeś:

```bash
uv run python -m robo_draw.main draw --svg-path examples/dudek.svg --setup sheet.json
```

Ten sam plik w układzie już stojącym w stacji, jeden do jednego, bez ponownego liczenia skali:

```bash
uv run python -m robo_draw.main draw --svg-path examples/dudek.svg --scale 1
```

Własny plik z Inkscape albo z `cloud-point-mesh` podstawiasz w `--svg-path`. Zapis z `cloud-point-mesh` jest zwykłym SVG ze ścieżkami, więc wchodzi tym samym poleceniem. Żeby obraz nie wyjechał poza papier, dodaj `--setup`.

## Prawdziwe ramię

Najpierw ten sam plik w symulacji. Obejrzyj, czy pisak zostaje nad stołem w przelotach i czy prostokąt rysunku leży na kartce.

Potem, przy RoboDK nadal ustawionym na symulację:

```bash
uv run python -m robo_draw.main draw \
  --svg-path examples/dudek.svg \
  --setup sheet.json \
  --run-on-robot
```

Program łączy się ze sterownikiem i przełącza RoboDK na ruch robota. Gdy stacja nie jest w symulacji, to polecenie się zatrzymuje. `--force-robot` tę odmowę pomija. Używaj go tylko wtedy, gdy świadomie chcesz ruszyć ramieniem przy RoboDK już przestawionym na robota.

`setup` nie ma trybu prawdziwego ramienia. Trzy punkty naucz w symulacji, dojeżdżając modelem do miejsc, które odpowiadają kartce, albo jogiem na żywo jeszcze przed wywołaniem programu i zatwierdzając Enter, gdy czubek już stoi. Samo `setup` wysyła do RoboDK tylko odczyt pozycji i zapis układu, nie sekwencję rysowania.

## Kółko i krzyżyk z kamerą

OnRobot Eyes to kamera RGB na USB-C, wpięta w Eye Box. Eye Box liczy własne detekcje przez Web Client i URCap, bez API do tej gry. Program czyta zwykły obraz przez OpenCV: numer kamery, gdy system widzi Eyes albo inną kamerę jako urządzenie USB, albo adres strumienia, gdy obraz jest wystawiony pod URL.

Gra używa kartki z `setup`. Robot rysuje siatkę, ty stawiasz X, po Enterze kamera sprawdza, które pole przybyło, i robot rysuje O. Pole 1 leży przy nauczonym rogu, 2 i 3 idą wzdłuż osi X, kolejne rzędy w stronę osi Y.

```bash
uv run python -m robo_draw.main tictactoe --setup sheet.json --camera 0
```

W oknie kamery kliknij cztery rogi narysowanej planszy w tej samej kolejności co przy uczeniu kartki: róg początkowy, koniec X, róg przeciwny, koniec Y. Enter w oknie zatwierdza kliknięcia, `r` kasuje punkty.

Potem w terminalu: narysuj jeden krzyżyk i naciśnij Enter. Program wypisuje, ile tuszu widzi w każdym polu. Czerwona ramka w oknie to pole uznane za zajęte. Gdy widok jest niepewny, wpisz numer pola `1`–`9` zamiast polegać na zdjęciu. `q` kończy grę.

Inna kamera albo strumień:

```bash
uv run python -m robo_draw.main tictactoe --setup sheet.json --camera 1
uv run python -m robo_draw.main tictactoe --setup sheet.json --camera http://192.168.1.50:8080/video
```

`--ink 0.045` to próg: udział ciemnych pikseli w środku pola. Jaśniejszy pisak wymaga mniejszej liczby, cień na kartce większej.

## API

Serwer przyjmuje SVG z innego komputera. Nie wczytuje `sheet.json` i nie dopasowuje skali do kartki. Układ i narzędzie muszą już stać w stacji. Skala z formularza jest używana wprost, domyślnie 1.

```bash
uv run python -m robo_draw.main serve --host 0.0.0.0 --port 8000
```

Opis endpointów jest też pod `http://127.0.0.1:8000/docs`.

Połączenie z symulatorem:

```bash
curl -X POST http://127.0.0.1:8000/connect \
  -F robot_name=Hanwha\ HCR-3A \
  -F run_on_robot=false \
  -F force_robot=false
```

Rysowanie w tle. Od razu wraca odpowiedź, a ramię jedzie dalej. Stan sprawdzisz przez `/status`.

```bash
curl -X POST http://127.0.0.1:8000/draw \
  -F svg_file=@examples/dudek.svg \
  -F scale=1 \
  -F resolution=1 \
  -F approach_dist=10
```

To samo, ale curl czeka do końca rysunku:

```bash
curl -X POST http://127.0.0.1:8000/draw/sync \
  -F svg_file=@examples/dudek.svg \
  -F approach_dist=10
```

Status i powrót do pozycji domowej:

```bash
curl http://127.0.0.1:8000/status
curl -X POST http://127.0.0.1:8000/home
```

`POST /stop` jest w serwerze, ale jeszcze nie przerywa ruchu. Żeby stanąć, użyj stopu w RoboDK.

Wysokość podnoszenia w API też jest w milimetrach nad papierem. Program bierze wartość bezwzględną, więc `-10` i `10` podnoszą pisak o 10 mm. Przy CLI trzymaj dodatnie 10, bo tak zapisuje się `approach_dist` w JSON-ie.

Drugi `POST /draw` w trakcie rysowania dostaje odpowiedź 409.

## Gdy coś nie wychodzi

**Wszystkie trzy punkty to `0.0, 0.0, 0.0 mm`.** Odczyt złapał bazę robota albo czubek naprawdę stoi w zerze stacji. Przesuń ramię jogiem i sprawdź, czy kolejne liczby na konsoli się zmieniają. Baza w zerze jest w porządku.

**`Punkty kartki są bliżej niż 1 mm`.** Drugi punkt jest przy rogu. Dojedź do końca dolnej krawędzi, nie do miejsca tuż obok pierwszego.

**`Punkty kartki leżą na jednej linii`.** Trzeci punkt jest na tej samej prostej co dwa pierwsze. Odsuń go w głąb kartki, prostopadle do dolnej krawędzi.

**`Frame item 'Drawing Frame' not found`.** Rysujesz bez `--setup`, a w stacji nie ma układu o tej nazwie. Uruchom `setup` albo wskaż istniejący układ przez `--frame-item-name`.

**Rysunek jest maleńki albo wychodzi poza papier.** Z `--setup` rozmiar kartki to odległość od rogu do dwóch końców krawędzi. Punkt w połowie krawędzi daje kartkę o połowę za małą i skala schodzi w dół. Naucz końce krawędzi jeszcze raz.

**Pisak zjeżdża w stół przy przelocie.** Lot ma iść w dodatnie Z układu kartki. Powtórz `setup`, żeby Z było w górę, i trzymaj `approach_dist` dodatnie. W CLI domyślnie jest 10.

**Połączenie z RoboDK odrzucone.** RoboDK ma być uruchomiony na tym samym komputerze. Port API to 20500. Z WSL adres `127.0.0.1` to Linux, nie Windows, więc to polecenie odpalaj w Windowsie.

**`Robot connection failed` przy `--run-on-robot`.** Sterownik nie jest w stanie gotowości. Zostaw próbę na symulacji, dopóki RoboDK nie pokazuje połączenia ze sterownikiem.

## Pomoc w terminalu

```bash
uv run python -m robo_draw.main --help
uv run python -m robo_draw.main setup --help
uv run python -m robo_draw.main draw --help
uv run python -m robo_draw.main serve --help
```
