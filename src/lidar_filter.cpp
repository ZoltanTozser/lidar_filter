// A program futtatásához azükséges include állományok. Ezek javarészt matematikai függvények gyűjteménye, 
// ROS-hoz, pointcloud-ok, marker-ek, dynamic reconfigure használatához szükséges állományok.   
#include <iostream>
#include <cmath>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <lidar_filter/dynamic_reconfConfig.h>


// GLOBÁLIS VÁLTOZÓK

// A topic név (amire felíratkozunk) és fixed frame tárolásához szükséges string változók.
std::string topic_name;
std::string fixed_frame;

// Az alkalmazott LIDAR csatornaszáma.
int channels = 64;

// A vizsgált területhez szükséges változók (X, Y, Z koordináta szerint a minimális és maximális közötti rész).
float min_x; 
float max_x;
float min_y;
float max_y;
float min_z;
float max_z;

// A LIDAR vertikális szögfelbontásának intervalluma.
float interval;

// A becsült pontok száma a járdaszegélyen.
int curb_points;

// A becsült minimális járdaszegély magasság.
float curb_height;

// Három pont által bezárt szög, X = 0 érték mellett.
float angle_filter1;

// Két vektor által bezárt szög, Z = 0 érték mellett.
float angle_filter2;

// A vizsgált sugárzóna mérete.
float beam_zone;

// Merőleges távolságméréshez szükséges küszöbérték (Lang algoritmusnál a megfelelő érték: 0.3). 
float epsilon;


// PARAMÉTEREK BEÁLLÍTÁSÁHOZ SZÜKSÉGES FÜGGVÉNY

// (Az értékeket a cfg könyvtárban található dynamic_reconf.cfg nevű fájlból veszi).
void params_callback(lidar_filter::dynamic_reconfConfig &config, uint32_t level)
{
    fixed_frame = config.fixed_frame;
    topic_name = config.topic_name;
    epsilon = config.epsilon;
    min_x = config.min_x;
    max_x = config.max_x;
    min_y = config.min_y;
    max_y = config.max_y;
    min_z = config.min_z;
    max_z = config.max_z;
    interval = config.interval;
    curb_points = config.curb_points;
    curb_height = config.curb_height;
    angle_filter1 = config.angle_filter1;
    angle_filter2 = config.angle_filter2;
    beam_zone = config.beam_zone;
}

 
// GYORSRENDEZŐ SEGÉDFÜGGVÉNYEK 

void swap(float *a, float *b)
{
    float temp = *a;
    *a = *b;
    *b = temp;
}

int partition(float ***arr_3d, int arc, int piece, int low, int high)
{
    float pivot = arr_3d[arc][low][4];
    int i = low - 1;
    int j = high + 1;

    while (true)
    {
        do
        {
            i++;
        } while (arr_3d[arc][i][4] < pivot);

        do
        {
            j--;
        } while (arr_3d[arc][j][4] > pivot);

        if (i >= j)
            return j;

        for (int sw = 0; sw < 7; sw++)
            {
                swap(&arr_3d[arc][i][sw], &arr_3d[arc][j][sw]);
            }
    }
}

void quicksort(float ***arr_3d, int arc, int piece, int low, int high)
{
    if (low < high)
    {
        int pi = partition(arr_3d, arc, piece, low, high);
        quicksort(arr_3d, arc, piece, low, pi);
        quicksort(arr_3d, arc, piece, pi + 1, high);
    }
}


// MERŐLEGES TÁVOLSÁGMÉRŐ FÜGGVÉNY

float perpendicular_distance(float ax, float ay, float bx, float by, float point_x, float point_y)
{
    float dx = bx - ax;
    float dy = by - ay;

    // Normalizálás
    float mag = sqrt(pow(dx, 2) + pow(dy, 2));
    
    if (mag > 0.0)
    {
        dx /= mag; 
        dy /= mag;
    }

    float pv_x = point_x - ax;
    float pv_y = point_y - by;

    float pv_dot = dx * pv_x + dy * pv_y;

    // Léptékvonal irányvektor
    float ds_x = pv_x * dx;
    float ds_y = pv_y * dy;

    float a_x = pv_x - ds_x;
    float a_y = pv_y - ds_y;

    return sqrt(pow(a_x, 2) + pow(a_y, 2));
}


// PUBLISHER-EK LÉTREHOZÁSA

// A vizsgált terület pontjai (Frame)
ros::Publisher pub_frame;

// Nem út pontok 
ros::Publisher pub_non_road;

// Út pontok
ros::Publisher pub_road;

// Marker array 
ros::Publisher pub_marker_array;


// A SZŰRÉST VÉGZŐ FÜGGVÉNY

void filter(const pcl::PointCloud<pcl::PointXYZ> &msg)
{
    // Segédváltozók a ciklusokhoz a függvényben.
    int i, j, k, l;

    // Egy pont tárolásához szükséges. 
    pcl::PointXYZ pt;
    
    // A vizsgált terület pontjainak tárolásához szükséges pontfelhő létrehozása.
    pcl::PointCloud<pcl::PointXYZ> filtered_frame;
    
    // Nem út pontok tárolásához szükséges pontfelhő létrehozása.
    pcl::PointCloud<pcl::PointXYZ> filtered_non_road;
    
    // Út pontok tárolásához szükséges pontfelhő létrehozása.
    pcl::PointCloud<pcl::PointXYZ> filtered_road;


    // VIZSGÁLT PONTOK KERESÉSE ÉS HOZZÁADÁSA A FILTERED_FRAME-HEZ

    // A for ciklussal végigmegyünk a bejövő (msg) pontfelhőn (az összes ponton).
    // Amelyik pont megfelel az if feltételnek (azaz a frame méreten belül elhelyezkedő pontok),
    // azokat hozzáadjuk a filtered_frame topic-hoz (push.back()).  
    for (i = 0; i <= msg.size(); i++)
    {
        if (msg.points[i].x >= min_x && msg.points[i].x <= max_x &&
            msg.points[i].y >= min_y && msg.points[i].y <= max_y &&
            msg.points[i].z >= min_z && msg.points[i].z <= max_z && 
            msg.points[i].x + msg.points[i].y + msg.points[i].z != 0)
            {
                pt.x = msg.points[i].x;
                pt.y = msg.points[i].y;
                pt.z = msg.points[i].z;
                filtered_frame.push_back(pt);
            }
    }
    
    // Meghatározzuk a pontok darabszámát és egy változóban tároljuk (ezt később több helyen is felhasználjuk majd). 
    int piece = filtered_frame.points.size();
    
    // Feltételként megadjuk, hogy legalább 40 pont legyen a filtered_frame-ben, azaz a vizsgált területen. 
    // Kevés pontot vizsgálni nincs értelme. 
    if (piece >= 40)
    {
        // DINAMIKUS 2D TÖMB LÉTREHOZÁSA A PONTOK ÉRTÉKEIHEZ ÉS EGYÉB SZÁMÍTÁSOKHOZ

        // 0. oszlop tárolja a pontok X értékét, 1. oszlop tárolja a pontok Y értékét, 
        // 2. oszlop tárolja a pontok Z értékét, 3. oszlop tárolja a pont és az origótól vett távolságot (D),
        // 4. oszlop tárolja a pontok szögfelbontását (Alfa).
        float **arr_2d = new float*[piece]();
        
        for (i = 0; i < piece; i++)
        {
            arr_2d[i] = new float[5];
        }
    
        // DINAMIKUS 2D TÖMB FELTÖLTÉSE ADATOKKAL

        // A szögfüggvényeknél a részeredmények tárolásához szükséges változó.
        float part_result;

        // Egy tömb, amiben eltároljuk a különböző szögfelbontásokat. Ez megegyezik a LIDAR csatornaszámával. 
        // A tömböt feltöltjük nulla értékekkel. 
        float angle[channels] = {0};
        
        // A szögfelbontásokat tároló tömb feltöltéséhez szükséges változó. 
        int index = 0;

        // Az adott szög új körvonalhoz tartozik vagy sem segédváltozója?
        int new_circle;

        // Végigmegyünk az összes ponton és feltöltjük az értékekkel a tömb oszlopait (X, Y, Z).
        // Kiszámítjuk az origótól vett távolságát (D) és a szögfelbontását (Alfa). 
        for (i = 0; i < piece; i++)
        {
            arr_2d[i][0] = filtered_frame.points[i].x;
            arr_2d[i][1] = filtered_frame.points[i].y;
            arr_2d[i][2] = filtered_frame.points[i].z;
            
            arr_2d[i][3] = sqrt(pow(arr_2d[i][0], 2) + pow(arr_2d[i][1], 2) + pow(arr_2d[i][2], 2));
            
            part_result = abs(arr_2d[i][2]) / arr_2d[i][3];

            // Kerekítési problémák miatt szükségesek az alábbi sorok.    
            if (part_result < -1)
                part_result = -1;

            else if (part_result > 1)
                part_result = 1;

            // Ha a Z értéke kisebb nullánál, akkor koszinusz függvényt kell alkalmazni. 
            if (arr_2d[i][2] < 0)  
                arr_2d[i][4] = acos(part_result) * 180 / M_PI;

            // Ha a Z értéke nagyobb vagy egyenlő nullánál, akkor pedig szinusz függvényt kell alkalmazni.     
            else if (arr_2d[i][2] >= 0)
                arr_2d[i][4] = (asin(part_result) * 180 / M_PI) + 90;
                

            // Az alapvetés az, hogy az adott szög új körvonalhoz tartozik. 
            new_circle = 1;

            for (j = 0; j < channels; j++)
            {
                if (angle[j] == 0)
                break;

                // Ha már korábban volt ilyen érték (egy meghatározott intervallumon belül), akkor ez nem egy új körív.
                // A new_Circle-be így nulla kerül és break-kel kilépünk a folyamatból. 
                if (abs(angle[j] - arr_2d[i][4]) <= interval)
                {
                    new_circle = 0;
                    break;
                }
            }             

            // Amennyiben nem szerepel még a tömbben ilyen érték, akkor az egy új körívet jelent. 
            if (new_circle == 1)
            {
                // Ha több körív keletkezne, mint 64 valamilyen okból kifolyólag, akkor hiba keletkezne. 
                // Az alábbi feltétel ezt kezeli le, illetve eltároljuk a különböző szögfelbontásokat.   
                if(index < channels)
                {
                    angle[index] = arr_2d[i][4];
                    index++;
                }
            }
        } 

        // A sort függvénnyel növekvő sorrendbe rendezzük a szögfelbontásokat. 
        std::sort(angle, angle + index);


        // DINAMIKUS 3D TÖMB LÉTREHOZÁSA A PONTOK ÉRTÉKEIHEZ ÉS EGYÉB SZÁMÍTÁSOKHOZ 

        // 0. oszlop tárolja a pontok X értékét, 1. oszlop tárolja a pontok Y értékét, 
        // 2. oszlop tárolja a pontok Z értékét, 3. oszlop tárolja a pont és az origótól vett távolságot, de itt Z = 0 értékkel (D),
        // 4. oszlop tárolja a pontok helyzetét egy körben (forgásszög számítása) (Alfa), 
        // 5. oszlop tárolja X = 0 érték mellett az új Y koordinátákat (új Y), 
        // 6. oszlop tárolja a csoportszámokat, ami lehet 1 (út pontot jelent), vagy 2 (nem út pontot jelent).
        float ***arr_3d = new float**[channels]();
 
        for (i = 0; i < channels; i++)
        {
            arr_3d[i] = new float*[piece];

            for (j = 0; j < piece; j++)
            {
                arr_3d[i][j] = new float[7];
            }
        }

        // DINAMIKUS 3D TÖMB FELTÖLTÉSE ADATOKKAL

        // Az adott köríveket tartalmazó csoportok (channels), megfelelő sorindexeinek beállításához szükséges tömb.
        // Nulla értékkel fel kell tölteni. 
        int index_array[channels] = {0};

        // Egy tömb, ami tárolja az adott köríven a legnagyobb távolságra lévő pont értékét az origótól.
        // Tehát minden köríven egy pont értékét tárolja a tömbben. Nulla értékkel fel kell tölteni.  
        float max_distance[channels] = {0};

        // Hibás LIDAR csatornaszám esetén szükséges változó.
        int results;
        
        // Végigmegyünk az összes ponton a for ciklussal. 
        for (i = 0; i < piece; i++)
        {
            results = 0;

            // Kiválasszuk a megfelelő körívet. 
            for (j = 0; j < index; j++)
            {
                if (abs(angle[j] - arr_2d[i][4]) <= interval)
                {
                    results = 1;
                    break;
                }
            }

            if (results == 1)
            {
                // A 2D tömbből hozzáadjuk az X, Y, Z koordináta értékeit a 3D tömbhöz (a megfelelő körívhez). 
                arr_3d[j][index_array[j]][0] = arr_2d[i][0];
                arr_3d[j][index_array[j]][1] = arr_2d[i][1];
                arr_3d[j][index_array[j]][2] = arr_2d[i][2];

                // A 2D tömbből kiszámítjuk az origótól vett távolságát, de itt csak az X és Y értéket adjuk hozzá. 
                arr_3d[j][index_array[j]][3] = sqrt(pow(arr_2d[i][0], 2) + pow(arr_2d[i][1], 2));

                // Minden pontnak van egy szöge 360 fokban. Az adott pont helyzete egy körben. 
                part_result = (abs(arr_3d[j][index_array[j]][0])) / arr_3d[j][index_array[j]][3];
            
                // Kerekítési problémák miatt szükségesek az alábbi sorok.    
                if (part_result < -1)
                    part_result = -1;

                else if (part_result > 1)
                    part_result > 1;

                // A pont a kör I. negyedében található
                if (arr_3d[j][index_array[j]][0] >= 0 && arr_3d[j][index_array[j]][1] <= 0)
                    arr_3d[j][index_array[j]][4] = asin(part_result) * 180 / M_PI;

                // A pont a kör II. negyedében található
                else if (arr_3d[j][index_array[j]][0] >= 0 && arr_3d[j][index_array[j]][1] > 0)
                    arr_3d[j][index_array[j]][4] = 180 - (asin(part_result) * 180 / M_PI);

                // A pont a kör III. negyedében található
                else if (arr_3d[j][index_array[j]][0] < 0 && 0 && arr_3d[j][index_array[j]][1] >= 0)
                    arr_3d[j][index_array[j]][4] = 180 + (asin(part_result) * 180 / M_PI);

                // A pont a kör IV. negyedében található
                else
                    arr_3d[j][index_array[j]][4] = 360 - (asin(part_result) * 180 / M_PI);
            
                // Vizsgálja, hogy az adott köríven az origótól a legnagyobb távolságra van-e a pont. 
                // Ha igen, akkor annak értékét elmenti.  
                if (arr_3d[j][index_array[j]][3] > max_distance[j])
                    max_distance[j] = arr_3d[j][index_array[j]][3];

                index_array[j]++;
            }
        }


        // 2D DINAMIKUS TÖMB FELSZABADÍTÁSA

        for (i = 0; i < piece; i++)
        {
            delete [] arr_2d[i];
        }
        delete [] arr_2d;

        
        // NEM ÚT PONTOK SZŰRÉSE

        // A három vizsgált pontból a második és a harmadik pont segédváltozója. 
        int point_2, point_3;

        // A két szélső pont közötti távolság. A LIDAR forgása és a körív szakadások miatt. 
        float d;

        // A három pont által bezárt háromszög oldalainak hossza. 
        float x1, x2, x3;

        // A három pont és a két vektor által bezárt szög. 
        float alpha; 

        // A két vektor változói. 
        float va1, va2, vb1, vb2;

        // A magasságot is kell vizsgálni nem csak a szöget. 
        float max1, max2; 

        // Végig iterálunk az összes körön.
        for ( i = 0; i < index; i++)
        {
            // FILTER 1

            // Feltöltjük új Y értékekkel a 6. oszlopot. 
            for (j = 1; j < index_array[i]; j++)
            {
                arr_3d[i][j][5] = arr_3d[i][j-1][5] + 0.0100;
            }

            // A adott kör pontjainak vizsgálata, X = 0 érték mellett.  
            for (j = curb_points; j <= (index_array[i] - 1) - curb_points; j++)
            {
                point_2 = j + curb_points / 2;
                point_3 = j + curb_points;

                // Kiszámoljuk a két szélső pont közötti távolságot. 
                d = sqrt(
                    pow(arr_3d[i][point_3][0] - arr_3d[i][j][0], 2) + 
                    pow(arr_3d[i][point_3][1] - arr_3d[i][j][1], 2));
                
                // Feltételhez kötjük, hogy a két szélsző pont közötti távolság kissebbnek kell lenni 5 méternél. 
                if (d < 5.0000)
                {
                    // Meghatározzuk a három pont által bezárt háromszög oldalainak hosszát (x1, x2, x3).
                    x1 = sqrt(
                        pow(arr_3d[i][point_2][5] - arr_3d[i][j][5], 2) + 
                        pow(arr_3d[i][point_2][2] - arr_3d[i][j][2], 2));
     
                    x2 = sqrt(
                        pow(arr_3d[i][point_3][5] - arr_3d[i][point_2][5], 2) + 
                        pow(arr_3d[i][point_3][2] - arr_3d[i][point_2][2], 2));

                    x3 = sqrt(
                        pow(arr_3d[i][point_3][5] - arr_3d[i][j][5], 2) + 
                        pow(arr_3d[i][point_3][2] - arr_3d[i][j][2], 2));
                    
                    part_result = (pow(x3, 2) - pow(x1, 2) - pow(x2, 2)) / (-2 * x1 * x2);
                    
                    // Kerekítési problémák miatt szükségesek az alábbi sorok.
                    if (part_result < -1)
                        part_result = -1;
                    else if (part_result > 1)
                        part_result = 1;

                    // Kiszámítjuk a három pont és a két vektor által bezárt szöget. 
                    alpha = acos(part_result) * 180 / M_PI;

                    // Ha a feltétel teljesül, akkor a 7. oszlopba 2-es szám kerül, azaz nem út pont. 
                    if (alpha <= angle_filter1 && 
                    (abs(arr_3d[i][j][2] - arr_3d[i][point_2][2]) >= curb_height ||
                    abs(arr_3d[i][point_3][2] - arr_3d[i][point_2][2]) >= curb_height) &&
                    abs(arr_3d[i][j][2] - arr_3d[i][point_3][2]) >= 0.05)
                    {
                        arr_3d[i][point_2][6] = 2;
                    }
                }
            }
            

            // FILTER 2

            // A adott kör pontjainak vizsgálata. Z = 0 érték mellett. 
            for (j = curb_points; j <= (index_array[i] - 1) - curb_points; j++)
            {
                // Kiszámoljuk a két vektor szélső pont közötti távolságot. 
                d = sqrt(
                    pow(arr_3d[i][j + curb_points][0] - arr_3d[i][j - curb_points][0], 2) +
                    pow(arr_3d[i][j + curb_points][1] - arr_3d[i][j - curb_points][1], 2));
                
                // Feltételhez kötjük, hogy a két vektor szélsző pont közötti távolság kissebbnek kell lenni 5 méternél. 
                if (d < 5.0000)
                {
                    // Kezdeti értékek beállítása. A max változókba bekerül Z abszolút értéke.
                    max1 = abs(arr_3d[i][j][2]);
                    max2 = abs(arr_3d[i][j][2]);
                    va1 = 0, va2 = 0;
                    vb1 = 0, vb2 = 0;

                    // Az 'a' vektor és a legnagyobb magasság meghatározása. 
                    for (k = j - 1;k >= j - curb_points; k--)
                    {
                        va1 = va1 + arr_3d[i][k][0] - arr_3d[i][j][0];
                        va2 = va2 + arr_3d[i][k][1] - arr_3d[i][j][1];

                        if (abs(arr_3d[i][k][2]) > max1)
                            max1 = abs(arr_3d[i][k][2]);
                    }

                    // A 'b' vektor és a legnagyobb magasság meghatározása.
                    for (k = j + 1;k <= j + curb_points; k++)
                    {
                        vb1 = vb1 + arr_3d[i][k][0] - arr_3d[i][j][0];
                        vb2 = vb2 + arr_3d[i][k][1] - arr_3d[i][j][1];

                        if (abs(arr_3d[i][k][2]) > max2)
                            max2 = abs(arr_3d[i][k][2]);
                    }

                    va1 = (1 / (float)curb_points) * va1;
                    va2 = (1 / (float)curb_points) * va2;
                    vb1 = (1 / (float)curb_points) * vb1;
                    vb1 = (1 / (float)curb_points) * vb1;

                    part_result = (va1 * vb1 + vb1 * vb2) / (sqrt(pow(va1, 2) + pow(va2, 2)) * sqrt(pow(vb1, 2) + pow(vb2, 2)));

                    // Kerekítési problémák miatt szükségesek az alábbi sorok.
                    if (part_result < - 1)
                        part_result = -1;
                    
                    if (part_result > 1)
                        part_result = 1;
                    
                    // Kiszámítjuk a két vektor által bezárt szöget. 
                    alpha = acos(part_result) * 180 / M_PI;

                    // Ha a feltétel teljesül, akkor a 7. oszlopba 2-es szám kerül, azaz nem út pont. 
                    // Ha a szög kisebb vagy egyenlő mint a beállított érték (alapesetben ez 140 fok) és
                    // a beállított küszöbértéktől (alapesetben ez 5cm) nagyobb vagy egyenlő Z értéke és
                    // a max1, max2 különbsége (abszolút értékben) nagyobb vagy egyenlő mint 0.05
                    if (alpha <= angle_filter2 && 
                        (max1 - abs(arr_3d[i][j][2]) >= curb_height ||
                         max2 - abs(arr_3d[i][j][2]) >= curb_height) && 
                         abs(max1 - max2) >= 0.05)
                         {
                            arr_3d[i][j][6] = 2;
                         }
                }   
            }
        }


        // ÚT PONTOK SZŰRÉSE

        // A tömb elemeinek gyorsrendezése körönként a szögeknek megfelelő növekvő sorrendben. 
        for (i = 0; i < index; i++)
        {
            quicksort(arr_3d, i, piece, 0, index_array[i] - 1);
        }

        // A körív mérete a megadott foknál. Minden körön ugyanakkor körív méretet kell vizsgálni. 
        float arc_distance;

        // Amennyiben az adott köríven, az adott szakaszon található magaspont, 
        // akkor 1-es értéket vesz fel a változó, egyébként nullát. 
        int not_road;

        // Az aktuális köríven a szög nagysága. 
        float current_degree;

        // A körív méret meghatározása. ((sugár * PI) / 180) * vizsgált sugárzóna mérettel).
        arc_distance = ((max_distance[0] * M_PI) / 180) * beam_zone;

        // 0 foktól 360 fok - beam_zone-ig vizsgáljuk a ciklussal. 
        for (i = 0; i <= 360 - beam_zone; i++)
        {   
            // Azt veszük alapul, hogy az adott szakaszon nincs magaspont. 
            not_road = 0;

            // Az első kör adott szakaszának vizsgálata. 
            for (j = 0; arr_3d[0][j][4] <= i + beam_zone && j < index_array[0]; j++)
            {
                if (arr_3d[0][j][4] >= i)
                {
                    // Nem vizsgáljuk tovább az adott szakaszt (break), mivel találtunk benne magaspontot. 
                    if (arr_3d[0][j][6] == 2)
                    {
                        not_road = 1;
                        break;
                    }
                }
            }

            // Amennyiben nem találtunk az első kör adott szakaszán magaspontot, akkor továbblépünk a következő körre. 
            if (not_road == 0)
            {
                // Az első kör szakaszát elfogadjuk, az adott pont tömb 7. oszlopába 1-es szám kerül, azaz út pont. 
                for (j = 0; arr_3d[0][j][4] <= i + beam_zone && j < index_array[0]; j++)
                {
                    if (arr_3d[0][j][4] >= i)
                    {
                        arr_3d[0][j][6] = 1;
                    }
                }

                // További körök vizsgálata. 
                for (k = 1; k < index; k++)
                {
                    // Új szöget kell meghatározni, hogy a távolabbi körvonalakon is ugyanakkora körív hosszt vizsgáljunk. 
                    if (i == 360 - beam_zone)
                    {
                        current_degree = 360;
                    }
                    else 
                    {
                        current_degree = i + arc_distance / ((max_distance[k] * M_PI) / 180);
                    }

                    // Az új kör pontjait vizsgáljuk. 
                    for (l = 0; arr_3d[k][l][4] <= current_degree && l < index_array[k]; l++)
                    {
                        if (arr_3d[k][l][4] >= i)
                        {
                            // Nem vizsgáljuk tovább az adott szakaszt (break), mivel találtunk benne magaspontot. 
                            if (arr_3d[k][l][6] == 2)
                            {
                                not_road = 1;
                                break;
                            }
                        }
                    }

                    // Ha a sugár elakadt egy magasponton, akkor a többi kört nem vizsgáljuk (break). 
                    if (not_road == 1)
                        break;
                    
                    // Az adott kör szakaszát elfogadjuk, az adott pont tömb 7. oszlopába 1-es szám kerül, azaz út pont. 
                    for (l = 0; arr_3d[k][l][4] <= current_degree && l < index_array[k]; l++)
                    {
                        if (arr_3d[k][l][4] >= i)
                        {
                            arr_3d[k][l][6] = 1;
                        }
                    }
                }
            }
        }


        // A TOPIC-OK FELTÖLTÉSE

        // Végig iterálunk az összes körön.
        for (i = 0; i < index; i++)
        {
            // Végig iterálunk a köríven található összes ponton.
            for (j = 0; j < index_array[i]; j++)
            {
                // Nem út pontok hozzáadása a filtered_non_road-hoz,
                // amennyiben a tömb 7. oszlopában 2-es szám van. 
                if (arr_3d[i][j][6] == 2)
                {
                    pt.x = arr_3d[i][j][0];
                    pt.y = arr_3d[i][j][1];
                    pt.z = arr_3d[i][j][2];
                    filtered_non_road.push_back(pt);
                }

                // Út pontok hozzáadása a filtered_road-hoz,
                // amennyiben a tömb 7. oszlopában 2-es szám van. 
                else if (arr_3d[i][j][6] == 1)
                {
                    pt.x = arr_3d[i][j][0];
                    pt.y = arr_3d[i][j][1];
                    pt.z = arr_3d[i][j][2];
                    filtered_road.push_back(pt);
                }
            }
        }
    

        // LEGTÁVOLABBI ÚT PONT KERESÉSE ADOTT FOKBAN (MARKER PONTOK KERESÉSE)

        // Két dimenziós tömb létrehozása. A tömb első három oszlopa tartalmazza az adott pont X, Y, Z koordinátáját.
        // A negyedik oszlop 0-ás, vagy 1-es értéket vesz fel. Ha az adott fokban találunk nem út pontot, akkor
        // 1-es értéket kap, ha nem találunk, akkor pedig 0-ás értéket kap. 
        float marker_array_points[piece][4];

        // Adott fokban a legtávolabbi út pont (zöld pont) távolságát tárolja a változó. 
        float max_distance_road;

        // A markerek feltöltéséhez szükséges segédváltozó. 
        int c = 0;

        // Az adott pont azonosításához szükséges változók. Az id_1 jelenti a körvonal azonosítását (melyik).
        // Az id_2 jelenti a korvonalon elhelyezkedő pont azonosítását (hanyadik a körvonalon). 
        int id_1, id_2;

        // A vizsgált sávban van-e magaspont, vagy olyan pont, amit nem jelölt a program se útnak, se magaspontnak. 
        int red_points;

        // A for ciklussal megvizsgáljuk a pontokat fokonként (0 foktól egészen 360 fokig). 
        for (i = 0; i <= 360; i++)
        {
            id_1 = -1;
            id_2 = -1;
            max_distance_road = 0;
            red_points = 0;

            // A for ciklussal végig megyünk az összes körvonalon. 
            for (j = 0; j < index; j++)
            {
                // A for ciklussal végig megyünk az adott körvonal összes pontján. 
                for (k = 0; k < index_array[j]; k++)
                {
                    // Ha találunk az adott fokban nem út pontot (piros pont), akkor break utasítással kilépünk, 
                    // nem vizsgáljuk tovább, mert nem lesz utána már út pont. 
                    // A red_point változóba 1-es érték kerül. 
                    if (arr_3d[j][k][6] != 1 && arr_3d[j][k][4] >= i && arr_3d[j][k][4] < i + 1)
                    {
                        red_points = 1;
                        break;
                    }

                    // Ha adott fokban találunk út pontot (zöld pont), akkor megvizsgáljuk az adott pontnak az origótól vett távolságát.  
                    if (arr_3d[j][k][6] == 1 && arr_3d[j][k][4] >= i && arr_3d[j][k][4] < i + 1)
                    {
                        d = sqrt(pow(0 - arr_3d[j][k][0], 2) + pow(0 - arr_3d[j][k][1], 2));

                        // Ha az origótól vett távolsága nagyobb az adott pontnak, mint az eddig a max_distance_road változóban tárolt érték,
                        // akkor új értékként ez kerül eltárolsára. Emellett eltároljuk az adott pont azonosítóit is (körvonal, hanyadik pont)
                        if (d > max_distance_road)
                        {
                            max_distance_road = d;
                            id_1 = j;
                            id_2 = k;
                        }
                    }
                }

                // Ezzel a break utasítással nem vizsgáljuk a további körvonalakat, hanem a következő fok vizsgálatával folytatjuk. 
                if (red_points == 1)
                    break;
            }

            // Amennyiben megtaláltuk a legtávolabb lévő út pontot adott fokban, akkor a tömbhöz hozzáadjuk a pontot. 
            // Hozzáadjuk az adott pont X, Y, Z koordinátáját és a red_points értékét. A c változóval számoljuk a pontok darabszámát. 
            if (id_1 != -1 && id_2 != -1)
            {
                marker_array_points[c][0] = arr_3d[id_1][id_2][0];
                marker_array_points[c][1] = arr_3d[id_1][id_2][1];
                marker_array_points[c][2] = arr_3d[id_1][id_2][2];
                marker_array_points[c][3] = red_points;
                c++;
            }
        }


        // MARKER PONTHALMAZ EGYSZERŰSÍTÉSE LANG ALGORITMUSSAL 

        float simp_marker_array_points[c][4];
        int count = 0;
        int counter = 0;

        for (i = 0; i < 4; i++)
        {
            simp_marker_array_points[0][i] = marker_array_points[0][i];
        }

        for (i = 4; i <= c - 1; i = i + 4)
        {
            counter = 0;

            for (j = 1; j <= 3; j++)
            {
                float d = perpendicular_distance(marker_array_points[i - 4][0], marker_array_points[i - 4][1], 
                                                 marker_array_points[i][0], marker_array_points[i][1],
                                                 marker_array_points[i - j][0], marker_array_points[i - j][1]);
                if (d > epsilon)
                {
                    counter++;
                    break;
                }

                if (j == 3 && d < epsilon)
                {
                    simp_marker_array_points[count][0] = marker_array_points[i][0];             
                    simp_marker_array_points[count][1] = marker_array_points[i][1];
                    simp_marker_array_points[count][2] = marker_array_points[i][2];
                    simp_marker_array_points[count][3] = marker_array_points[i][3];
                    count++;
                }
            }
    
            if (counter == 1)
            {
                counter = 2;

                for (j = 2; j <= 3; j++)
                {
                    float d = perpendicular_distance(marker_array_points[i - 4][0], marker_array_points[i - 4][1], 
                                                     marker_array_points[i - 1][0], marker_array_points[i - 1][1],
                                                     marker_array_points[i - j][0], marker_array_points[i - j][1]);
                    if (d > epsilon)
                    {
                        counter++;
                        break;
                    }

                    if (j == 3 && d < epsilon)
                    {
                        simp_marker_array_points[count][0] = marker_array_points[i - 1][0];             
                        simp_marker_array_points[count][1] = marker_array_points[i - 1][1];
                        simp_marker_array_points[count][2] = marker_array_points[i - 1][2];
                        simp_marker_array_points[count][3] = marker_array_points[i - 1][3];
                        count++;
                        i = i - 1;
                    }
                }
            }
            
            if (counter == 3)
            {
                float d = perpendicular_distance(marker_array_points[i - 4][0], marker_array_points[i - 4][1], 
                                                 marker_array_points[i - 2][0], marker_array_points[i - 2][1],
                                                 marker_array_points[i - 3][0], marker_array_points[i - 3][1]);
                if (d > epsilon)
                {
                    simp_marker_array_points[count][0] = marker_array_points[i - 3][0];             
                    simp_marker_array_points[count][1] = marker_array_points[i - 3][1];
                    simp_marker_array_points[count][2] = marker_array_points[i - 3][2];
                    simp_marker_array_points[count][3] = marker_array_points[i - 3][3];
                    count++;
                    i = i - 3;
                }

                if (d < epsilon)
                {
                    simp_marker_array_points[count][0] = marker_array_points[i - 2][0];             
                    simp_marker_array_points[count][1] = marker_array_points[i - 2][1];
                    simp_marker_array_points[count][2] = marker_array_points[i - 2][2];
                    simp_marker_array_points[count][3] = marker_array_points[i - 2][3];
                    count++;
                    i = i - 2;
                }
            }
        }

        for (i = 0; i < 4; i++)
        {
            simp_marker_array_points[count][i] = marker_array_points[c - 1][i];
        } 
         
        // std::cout << count << std::endl;


        // MARKER ÖSSZEÁLLÍTÁSA

        if (count > 2)
        {
            if (simp_marker_array_points[0][3] = 0 && simp_marker_array_points[1][3] == 1)
                simp_marker_array_points[0][3] = 1;

            if (simp_marker_array_points[count - 1][3] == 0 && simp_marker_array_points[count - 2][3] == 1)
                simp_marker_array_points[count - 1][3] == 1;

            if (simp_marker_array_points[0][3] = 1 && simp_marker_array_points[1][3] == 0)
                simp_marker_array_points[0][3] = 0;

            if (simp_marker_array_points[count - 1][3] == 1 && simp_marker_array_points[count - 2][3] == 0)
                simp_marker_array_points[count - 1][3] = 0;

            for (i = 2; i <= count - 3; i++)
            {
                if (simp_marker_array_points[i][3] == 0 && simp_marker_array_points[i - 1][3] == 1 && simp_marker_array_points[i + 1][3] == 1)
                    simp_marker_array_points[i][3] = 1;
            }

            for (i = 2; i <= count - 3; i++)
            {
                if (simp_marker_array_points[i][3] == 1 && simp_marker_array_points[i - 1][3] == 0 && simp_marker_array_points[i + 1][3] == 0)
                    simp_marker_array_points[i][3] = 0;
            }

            visualization_msgs::MarkerArray marker_arr;
            visualization_msgs::Marker line_segment;
            geometry_msgs::Point point;          

            int line_segment_id = 0;

            line_segment.header.frame_id = fixed_frame;
            line_segment.header.stamp = ros::Time();
            line_segment.type = visualization_msgs::Marker::LINE_STRIP;
            line_segment.action = visualization_msgs::Marker::ADD;

            for (i = 0; i < count; i++)
            {
                point.x = simp_marker_array_points[i][0];
                point.y = simp_marker_array_points[i][1];
                point.z = simp_marker_array_points[i][2];
            
                if (i == 0)
                {
                    line_segment.points.push_back(point);
                }

                else if (simp_marker_array_points[i][3] == simp_marker_array_points[i - 1][3])
                {
                    line_segment.points.push_back(point);

                    if (i == count - 1)
                    {
                        line_segment.id = line_segment_id;

                        line_segment.pose.position.x = 0;
                        line_segment.pose.position.y = 0;
                        line_segment.pose.position.z = 0;

                        line_segment.pose.orientation.x = 0.0;
                        line_segment.pose.orientation.y = 0.0;
                        line_segment.pose.orientation.z = 0.0;
                        line_segment.pose.orientation.w = 1.0;

                        line_segment.scale.x = 0.5;
                        line_segment.scale.y = 0.5;
                        line_segment.scale.z = 0.5;
                    
                        if (simp_marker_array_points[i][3] == 0)
                        {
                            line_segment.color.a = 1.0;
                            line_segment.color.r = 0.0;
                            line_segment.color.g = 1.0;
                            line_segment.color.b = 0.0;
                        }

                        else
                        {
                            line_segment.color.a = 1.0;
                            line_segment.color.r = 1.0;
                            line_segment.color.g = 0.0;
                            line_segment.color.b = 0.0;
                        }

                        marker_arr.markers.push_back(line_segment);
                        line_segment.points.clear();
                    }
                }

                else if (simp_marker_array_points[i][3] != simp_marker_array_points[i - 1][3] && simp_marker_array_points[i][3] == 0)
                {
                    line_segment.points.push_back(point);

                    line_segment.id = line_segment_id;
                    line_segment_id++;

                    line_segment.pose.position.x = 0;
                    line_segment.pose.position.y = 0;
                    line_segment.pose.position.z = 0;

                    line_segment.pose.orientation.x = 0.0;
                    line_segment.pose.orientation.y = 0.0;
                    line_segment.pose.orientation.z = 0.0;
                    line_segment.pose.orientation.w = 1.0;

                    line_segment.scale.x = 0.5;
                    line_segment.scale.y = 0.5;
                    line_segment.scale.z = 0.5;

                    line_segment.color.a = 1.0;
                    line_segment.color.r = 1.0;
                    line_segment.color.g = 0.0;
                    line_segment.color.b = 0.0;

                    marker_arr.markers.push_back(line_segment);
                    line_segment.points.clear();
                    line_segment.points.push_back(point);
                }

                else if (simp_marker_array_points[i][3] != simp_marker_array_points[i - 1][3] && simp_marker_array_points[i][3] == 1)
                {
                    line_segment.points.push_back(point);

                    line_segment.id = line_segment_id;
                    line_segment_id++;

                    line_segment.pose.position.x = 0;
                    line_segment.pose.position.y = 0;
                    line_segment.pose.position.z = 0;

                    line_segment.pose.orientation.x = 0.0;
                    line_segment.pose.orientation.y = 0.0;
                    line_segment.pose.orientation.z = 0.0;
                    line_segment.pose.orientation.w = 1.0;

                    line_segment.scale.x = 0.5;
                    line_segment.scale.y = 0.5;
                    line_segment.scale.z = 0.5;

                    line_segment.color.a = 1.0;
                    line_segment.color.r = 0.0;
                    line_segment.color.g = 1.0;
                    line_segment.color.b = 0.0;

                    marker_arr.markers.push_back(line_segment);
                    line_segment.points.clear();
                
                    point.x = simp_marker_array_points[i - 1][0];
                    point.y = simp_marker_array_points[i - 1][1];
                    point.z = simp_marker_array_points[i - 1][2];
                    line_segment.points.push_back(point);
                    
                
                    point.x = simp_marker_array_points[i][0];
                    point.y = simp_marker_array_points[i][1];
                    point.z = simp_marker_array_points[i][2];
                    line_segment.points.push_back(point);
                }

                line_segment.lifetime = ros::Duration(0.04);
            }

            pub_marker_array.publish(marker_arr);
        }


        // 3D DINAMIKUS TÖMB FELSZABADÍTÁSA
        
        for (i = 0; i < channels; i++)
        {
            for (j = 0; j < piece; j++)
            {
                delete [] arr_3d[i][j];
            }
            delete [] arr_3d[i];
        }

        delete[] arr_3d;
    }
    
    filtered_frame.header = msg.header;
    pub_frame.publish(filtered_frame);
    
    filtered_non_road.header = msg.header;
    pub_non_road.publish(filtered_non_road);

    filtered_road.header = msg.header;
    pub_road.publish(filtered_road);   
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidarFilter");

    dynamic_reconfigure::Server<lidar_filter::dynamic_reconfConfig> server;
    dynamic_reconfigure::Server<lidar_filter::dynamic_reconfConfig>::CallbackType f;

    f = boost::bind(&params_callback, _1, _2);
    server.setCallback(f);

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe(topic_name, 1, filter);
    
    pub_frame = node.advertise<pcl::PCLPointCloud2>("filtered_frame", 1);
    pub_non_road = node.advertise<pcl::PCLPointCloud2>("filtered_non_road", 1);
    pub_road = node.advertise<pcl::PCLPointCloud2>("filtered_road", 1);
    pub_marker_array = node.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

    ros::spin();

    return 0;
}