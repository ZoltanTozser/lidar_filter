#include <iostream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <cmath>
#include <math.h>
#include <algorithm>


int channels = 64;

float interval = 0.1800;

int curb_points = 5;
double curb_height = 0.0500;
double angle_filter1 = 150;
double angle_filter2 = 140;

 float beam_zone = 30;
 int x_direction = 0;

float min_x = 0; 
float max_x = 30;
float min_y = -10;
float max_y = 10;
float min_z = -3;
float max_z = -1;

std::string topic_name = "/left_os1/os1_cloud_node/points";


ros::Publisher pub_frame;
ros::Publisher pub_non_road;
ros::Publisher pub_road;


// GYORSRENDEZŐ SEGÉDFÜGGVÉNYEK

void swap(float *a, float *b)
{
    float temp = *a;
    *a = *b;
    *b = temp;
}

int partition(float ***arr_3d, int arc, int piece, int low, int high)
{
    float pivot = arr_3d[arc][high][4];
    int i = (low - 1);

    for (int j = low; j <= high - 1; j++)
    {
        if (arr_3d[arc][j][4])
        {
            i++;
            for (int sw = 0; sw < 7; sw++)
            {
                swap(&arr_3d[arc][i][sw], &arr_3d[arc][j][sw]);
            }
        }
    }

    for (int sw = 0; sw < 7; sw++)
    {
        swap(&arr_3d[arc][i+1][sw], &arr_3d[arc][high][sw]);
    }

    return (i + 1);
}

void quicksort(float ***arr_3d, int arc, int piece, int low, int high)
{
    if (low < high)
    {
        int pi = partition(arr_3d, arc, piece, low, high);
        quicksort(arr_3d, arc, piece, low, pi - 1);
        quicksort(arr_3d, arc, piece, pi + 1, high);
    }
}


void filter(const pcl::PointCloud<pcl::PointXYZ> &msg)
{
    int i, j, k, l;

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> filtered_frame;
    pcl::PointCloud<pcl::PointXYZ> filtered_non_road;
    pcl::PointCloud<pcl::PointXYZ> filtered_road;


    // VIZSGÁLT PONTOK KERESÉSE ÉS HOZZÁADÁSA A FILTERED_FRAME-HEZ

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
    

    int piece = filtered_frame.points.size();
    
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


        float quarter1 = 0, quarter2 = 180, quarter3, quarter4 = 360; 
        int c1 = -1, c2 = -1, c3 = -1, c4 = -1;

        for (i = 0; i < index_array[1]; i++)
        {
            if (arr_3d[1][i][6] == 2)
                {
                    if (arr_3d[1][i][4] >= 0 && arr_3d[1][i][4] < 90)
                    {
                        if (arr_3d[1][i][4] > quarter1)
                        {
                            quarter1 = arr_3d[1][i][4];
                            c1 = i;
                        }
                    }

                    else if(arr_3d[1][i][4] >= 90 && arr_3d[1][i][4] < 180)
                    {
                        if (arr_3d[1][i][4] > quarter2)
                        {
                            quarter2 = arr_3d[1][i][4];
                            c2 = i;
                        }
                    }

                    else if(arr_3d[1][i][4] >= 180 && arr_3d[1][i][4] < 270)
                    {
                        if (arr_3d[1][i][4] > quarter3)
                        {
                            quarter3 = arr_3d[1][i][4];
                            c3 = i;
                        }
                    }

                    else 
                    {
                        if (arr_3d[1][i][4] > quarter4)
                        {
                            quarter4 = arr_3d[1][i][4];
                            c4 = i;
                        }
                    }
                }
        }


        float arc_distance;
        int not_road;
        int blind_spot;
        float current_degree;

        arc_distance = ((max_distance[0] * M_PI) / 180) * beam_zone;

        for (i = 0; i <= 360 - beam_zone; i++)
        {
            blind_spot = 0;

            if (x_direction == 0)
            {
                if ((quarter1 != 0 && quarter4 != 360 && (i <= quarter1 || i >= quarter4)) ||
                    (quarter2 != 180 && quarter3 != 180 && quarter3 != 180 && i >= quarter2 && i <= quarter3))
                    {
                        blind_spot = 1;
                    }
            }
            
            else if (x_direction == 1)
            {
                if ((quarter2 != 180 && i >= quarter2 && i <= 270) ||
                    (quarter1 != 0 && (i <= quarter1 || i >= 270)))
                    {
                        blind_spot = 1;
                    }
            }

            else
            {
                if ((quarter4 != 360 && (i >= quarter4 || i <= 90)) ||
                    (quarter3 != 180 && i <= quarter3 && i >= 90))
                    {
                        blind_spot = 1;
                    }
            }
        }

        if (blind_spot == 0)
        {
            not_road = 0;

            for (j = 0; arr_3d[0][j][4] <= i + beam_zone && j < index_array[0]; j++)
            {
                if (arr_3d[0][j][4] >= 1)
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
                    if (i == 360 -beam_zone)
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
                
                if (arr_3d[i][j][6] == 1)
                {
                    pt.x = arr_3d[i][j][0];
                    pt.y = arr_3d[i][j][1];
                    pt.z = arr_3d[i][j][2];
                    filtered_road.push_back(pt);
                }
            }
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

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe(topic_name, 1, filter);
    
    pub_frame = node.advertise<pcl::PCLPointCloud2>("filtered_frame", 1);
    pub_non_road = node.advertise<pcl::PCLPointCloud2>("filtered_non_road", 1);
    pub_road = node.advertise<pcl::PCLPointCloud2>("filtered_road", 1);

    ros::spin();

    return 0;
}