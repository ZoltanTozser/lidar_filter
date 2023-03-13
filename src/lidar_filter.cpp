// A PROGRAM FUTTATÁSÁHOZ SZÜKSÉGES INCLUDE ÁLLOMÁNYOK. EZEK JAVARÉSZT MATEMATIKAI FÜGGVÉNYEK GYŰJTEMÉNYE, 
// ROS-HOZ, POINT CLOUD-OK, MARKER-EK, DYNAMIC RECONFIGURE HASZNÁLATÁHOZ SZÜKSÉGES ÁLLOMÁNYOK.   
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

// A TOPIC NÉV (AMIRE FELÍRATKOZUNK) ÉS FIXED FRAME TÁROLÁSÁHOZ SZÜKSÉGES STRING VÁLTOZÓK.
std::string topic_name;
std::string fixed_frame;

// AZ ALKALMAZOTT LIDAR CSATORNASZÁMA.
int channels = 64;

// VIZSGÁLT TERÜLETHEZ SZÜKSÉGES VÁLTOZÓK (X, Y, Z KOORDINÁTA SZERINT A MINIMÁLIS ÉS MAXIMÁLIS KÖZÖTTI RÉSZ).
float min_x; 
float max_x;
float min_y;
float max_y;
float min_z;
float max_z;

// LIDAR VERTIKÁLIS SZÖGFELBONTÁSÁNAK INTERVALLUMA.
float interval;

// BECSÜLT PONTOK SZÁMA A JÁRDASZEGÉLYEN.
int curb_points;

// BECSÜLT MINIMÁLIS JÁRDASZEGÉLY MAGASSÁG.
float curb_height;

// HÁROM PONT ÁLTAL BEZÁRT SZÖG, X = 0 ÉRTÉK MELLETT.
float angle_filter1;

// KÉT VEKTOR ÁLTAL BEZÁRT SZÖG, Z = 0 ÉRTÉK MELLETT.
float angle_filter2;

// VIZSGÁLT SUGÁRZÓNA MÉRETE.
float beam_zone;

// MERŐLEGES TÁVOLSÁGMÉRÉSHEZ SZÜKSÉGES KÜSZÖBÉRTÉK (LANG ALGORITMUSNÁL A MEGFELEŐ ÉRTÉK: 0.3). 
float epsilon;


// PARAMÉTEREK BEÁLLÍTÁSÁHOZ SZÜKSÉGES FÜGGVÉNY 
// (AZ ÉRTÉKEKET A CFG KÖNYVTÁRBAN TALÁLHATÓ DYNAMIC_RECONF.CFG NEVŰ FÁJLBÓL VESZI).
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

    // NORMALIZÁLÁS
    float mag = sqrt(pow(dx, 2) + pow(dy, 2));
    
    if (mag > 0.0)
    {
        dx /= mag; 
        dy /= mag;
    }

    float pv_x = point_x - ax;
    float pv_y = point_y - by;

    float pv_dot = dx * pv_x + dy * pv_y;

    // LÉPTÉKVOLNAL IRÁNYVEKTOR
    float ds_x = pv_x * dx;
    float ds_y = pv_y * dy;

    float a_x = pv_x - ds_x;
    float a_y = pv_y - ds_y;

    return sqrt(pow(a_x, 2) + pow(a_y, 2));
}


// PUBLISHER-EK LÉTREHOZÁSA

// A VIZSGÁLT TERÜLET PONTJAI (FRAME)
ros::Publisher pub_frame;

// NEM ÚT PONTOK 
ros::Publisher pub_non_road;

// ÚT PONTOK
ros::Publisher pub_road;

// MARKER ARRAY 
ros::Publisher pub_marker_array;


// A SZŰRÉST VÉGZŐ FÜGGVÉNY
void filter(const pcl::PointCloud<pcl::PointXYZ> &msg)
{
    // SEGÉDVÁLTOZÓK A CIKLUSOKHOZ A FÜGGVÉNYBEN.
    int i, j, k, l;

    // EGY PONT TÁROLÁSÁHOZ SZÜKSÉGES 
    pcl::PointXYZ pt;
    
    // A VIZSGÁLT TERÜLET PONTJAINAK TÁROLÁSÁHOZ SZÜKSÉGES PONTFELHŐ LÉTREHOZÁSA
    pcl::PointCloud<pcl::PointXYZ> filtered_frame;
    
    // NEM ÚT PONTOK PONTJAINAK TÁROLÁSÁHOZ SZÜKSÉGES PONTFELHŐ LÉTREHOZÁSA
    pcl::PointCloud<pcl::PointXYZ> filtered_non_road;
    
    // ÚT PONTOK PONTJAINAK TÁROLÁSÁHOZ SZÜKSÉGES PONTFELHŐ LÉTREHOZÁSA
    pcl::PointCloud<pcl::PointXYZ> filtered_road;


    // VIZSGÁLT PONTOK KERESÉSE ÉS HOZZÁADÁSA A FILTERED_FRAME-HEZ

    // A FOR CIKLUSSAL VÉGIGMEGYÜNK A BEJÖVŐ (MSG) PONTFELHŐN (AZ ÖSSZES PONTON).
    // AMELYIK PONT MEGFLELEL AZ IF FELTÉTELNEK (AZAZ A FRAME MÉRETEN BELÜL ELHELYZKEDŐ PONTOK)
    // AZOKAT HOZZÁADJUK A FILTERED_FRAME TOPIC-HOZ (PUSH.BACK()).  
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
    
    // MEGHATÁROZZUK A PONTOK DARABSZÁMÁT ÉS EGY VÁLTOZÓBAN TÁROLJUK (EZT KÉSŐBB TÖBB HELYEN IS FELHASZNÁLJUK MAJD). 
    int piece = filtered_frame.points.size();
    
    // FELTÉTELKÉNT MEGADJUK, HOGY LEGALÁBB 40 PONT LEGYEN A FILTERED FRAME-BEN, AZAZ A VIZSGÁLT TERÜLETEN. 
    // KEVÉS PONTOT VIZSGÁLNI NINCS ÉRTELME. 
    if (piece >= 40)
    {
        // DINAMIKUS 2D TÖMB LÉTREHOZÁSA A PONTOK ÉRTÉKEIHEZ ÉS EGYÉB SZÁMÍTÁSOKHOZ 
        
        float **arr_2d = new float*[piece]();
        
        for (i = 0; i < piece; i++)
        {
            arr_2d[i] = new float[5];
        }
    
        // 2D TÖMB FELTÖLTÉSE ADATOKKAL

        float part_result;

        float angle[channels] = {0};
        
        int index = 0;

        int new_circle;

        for (i = 0; i < piece; i++)
        {
            arr_2d[i][0] = filtered_frame.points[i].x;
            arr_2d[i][1] = filtered_frame.points[i].y;
            arr_2d[i][2] = filtered_frame.points[i].z;
            
            arr_2d[i][3] = sqrt(pow(arr_2d[i][0], 2) + pow(arr_2d[i][1], 2) + pow(arr_2d[i][2], 2));
            
            part_result = abs(arr_2d[i][2]) / arr_2d[i][3];
                
            if (part_result < -1)
                part_result = -1;

            if (part_result > 1)
                part_result = 1;

            if (arr_2d[i][2] < 0)  
                arr_2d[i][4] = acos(part_result) * 180 / M_PI;
                
            else if (arr_2d[i][2] >= 0)
                arr_2d[i][4] = (asin(part_result) * 180 / M_PI) + 90;
                

            new_circle = 1;

            for (j = 0; j < channels; j++)
            {
                if (angle[j] == 0)
                break;

                if (abs(angle[j] - arr_2d[i][4]) <= interval)
                {
                    new_circle = 0;
                    break;
                }
            }             

            if (new_circle == 1)
            {
                if(index < channels)
                {
                    angle[index] = arr_2d[i][4];
                    index++;
                }
            }
        } 

        std::sort(angle, angle + index);


        // DINAMIKUS 3D TÖMB LÉTREHOZÁSA A PONTOK ÉRTÉKEIHEZ ÉS EGYÉB SZÁMÍTÁSOKHOZ 

        float ***arr_3d = new float**[channels]();
 
        for (i = 0; i < channels; i++)
        {
            arr_3d[i] = new float*[piece];

            for (j = 0; j < piece; j++)
            {
                arr_3d[i][j] = new float[7];
            }
        }


        // 3D TÖMB FELTÖLTÉSE ADATOKKAL

        int index_array[channels] = {0};

        float max_distance[channels] = {0};

        int results;
        
        for (i = 0; i < piece; i++)
        {
            results = 0;

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
                arr_3d[j][index_array[j]][0] = arr_2d[i][0];
                arr_3d[j][index_array[j]][1] = arr_2d[i][1];
                arr_3d[j][index_array[j]][2] = arr_2d[i][2];

                arr_3d[j][index_array[j]][3] = sqrt(pow(arr_2d[i][0], 2) + pow(arr_2d[i][1], 2));

                part_result = (abs(arr_3d[j][index_array[j]][0])) / arr_3d[j][index_array[j]][3];
            
                if (part_result < -1)
                    part_result = -1;

                else if (part_result > 1)
                    part_result > 1;

                if (arr_3d[j][index_array[j]][0] >= 0 && arr_3d[j][index_array[j]][1] <= 0)
                    arr_3d[j][index_array[j]][4] = asin(part_result) * 180 / M_PI;

                else if (arr_3d[j][index_array[j]][0] >= 0 && arr_3d[j][index_array[j]][1] > 0)
                    arr_3d[j][index_array[j]][4] = 180 - (asin(part_result) * 180 / M_PI);
            
                else if (arr_3d[j][index_array[j]][0] < 0 && 0 && arr_3d[j][index_array[j]][1] >= 0)
                    arr_3d[j][index_array[j]][4] = 180 + (asin(part_result) * 180 / M_PI);
            
                else
                    arr_3d[j][index_array[j]][4] = 360 - (asin(part_result) * 180 / M_PI);
            
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

        int point_2, point_3;
        float alpha; 
        float x1, x2, x3;
        float va1, va2, vb1, vb2;
        float max1, max2; 
        float d;

        for ( i = 0; i < index; i++)
        {
            // FILTER 1

            for (j = 1; j < index_array[i]; j++)
            {
                arr_3d[i][j][5] = arr_3d[i][j-1][5] + 0.0100;
            }

            for (j = curb_points; j <= (index_array[i] - 1) - curb_points; j++)
            {
                point_2 = j + curb_points / 2;
                point_3 = j + curb_points;

                d = sqrt(
                    pow(arr_3d[i][point_3][0] - arr_3d[i][j][0], 2) + 
                    pow(arr_3d[i][point_3][1] - arr_3d[i][j][1], 2));
                
                if (d < 5.0000)
                {
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
                    
                    if (part_result < -1)
                        part_result = -1;
                    else if (part_result > 1)
                        part_result = 1;

                    alpha = acos(part_result) * 180 / M_PI;

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

            for (j = curb_points; j <= (index_array[i] - 1) - curb_points; j++)
            {
                d = sqrt(
                    pow(arr_3d[i][j + curb_points][0] - arr_3d[i][j - curb_points][0], 2) +
                    pow(arr_3d[i][j + curb_points][1] - arr_3d[i][j - curb_points][1], 2));
                
                if (d < 5.0000)
                {
                    max1 = abs(arr_3d[i][j][2]);
                    max2 = abs(arr_3d[i][j][2]);
                    va1 = 0, va2 = 0;
                    vb1 = 0, vb2 = 0;

                    for (k = j - 1;k >= j - curb_points; k--)
                    {
                        va1 = va1 + arr_3d[i][k][0] - arr_3d[i][j][0];
                        va2 = va2 + arr_3d[i][k][1] - arr_3d[i][j][1];

                        if (abs(arr_3d[i][k][2]) > max1)
                            max1 = abs(arr_3d[i][k][2]);
                    }

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

                    if (part_result < - 1)
                        part_result = -1;
                    
                    if (part_result > 1)
                        part_result = 1;
                    
                    alpha = acos(part_result) * 180 / M_PI;

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

        for (i = 0; i < index; i++)
        {
            quicksort(arr_3d, i, piece, 0, index_array[i] - 1);
        }


        float arc_distance;
        int not_road;
        float current_degree;

        arc_distance = ((max_distance[0] * M_PI) / 180) * beam_zone;

        for (i = 0; i <= 360 - beam_zone; i++)
        {   
            not_road = 0;

            for (j = 0; arr_3d[0][j][4] <= i + beam_zone && j < index_array[0]; j++)
            {
                if (arr_3d[0][j][4] >= i)
                {
                    if (arr_3d[0][j][6] == 2)
                    {
                        not_road = 1;
                        break;
                    }
                }
            }

            if (not_road == 0)
            {
                for (j = 0; arr_3d[0][j][4] <= i + beam_zone && j < index_array[0]; j++)
                {
                    if (arr_3d[0][j][4] >= i)
                    {
                        arr_3d[0][j][6] = 1;
                    }
                }

                for (k = 1; k < index; k++)
                {
                    if (i == 360 - beam_zone)
                    {
                        current_degree = 360;
                    }
                    else 
                    {
                        current_degree = i + arc_distance / ((max_distance[k] * M_PI) / 180);
                    }

                    for (l = 0; arr_3d[k][l][4] <= current_degree && l < index_array[k]; l++)
                    {
                        if (arr_3d[k][l][4] >= i)
                        {
                            if (arr_3d[k][l][6] == 2)
                            {
                                not_road = 1;
                                break;
                            }
                        }
                    }

                    if (not_road == 1)
                        break;
                    
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

        
        // A CSOPORTOK FELTÖLTÉSE

        for (i = 0; i < index; i++)
        {
            for (j = 0; j < index_array[i]; j++)
            {
                // NEM ÚT PONTOK
                
                if (arr_3d[i][j][6] == 2)
                {
                    pt.x = arr_3d[i][j][0];
                    pt.y = arr_3d[i][j][1];
                    pt.z = arr_3d[i][j][2];
                    filtered_non_road.push_back(pt);
                }

                // ÚT PONTOK
                
                else if (arr_3d[i][j][6] == 1)
                {
                    pt.x = arr_3d[i][j][0];
                    pt.y = arr_3d[i][j][1];
                    pt.z = arr_3d[i][j][2];
                    filtered_road.push_back(pt);
                }
            }
        }
    

        // LEGTÁVOLABBI ÚT PONT KERESÉSE ADOTT FOKBAN (MARKER PONTOK)

        float marker_array_points[piece][4];
        float max_distance_road;
        int c = 0;
        int id_1, id_2;
        int red_points;

        for (i = 0; i <= 360; i++)
        {
            id_1 = -1;
            id_2 = -1;
            max_distance_road = 0;
            red_points = 0;
        

            for (j = 0; j < index; j++)
            {
                for (k = 0; k < index_array[j]; k++)
                {
                    if (arr_3d[j][k][6] != 1 && arr_3d[j][k][4] >= i && arr_3d[j][k][4] < i + 1)
                    {
                        red_points = 1;
                        break;
                    }

                    if (arr_3d[j][k][6] == 1 && arr_3d[j][k][4] >= i && arr_3d[j][k][4] < i + 1)
                    {
                        d = sqrt(pow(0 - arr_3d[j][k][0], 2) + pow(0 - arr_3d[j][k][1], 2));

                        if (d > max_distance_road)
                        {
                            max_distance_road = d;
                            id_1 = j;
                            id_2 = k;
                        }
                    }
                }

                if (red_points == 1)
                    break;
            }


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