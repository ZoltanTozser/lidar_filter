#include <iostream>
#include <cmath>
#include <math.h>
#include <algorithm>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

std::string topic_name = "/left_os1/os1_cloud_node/points";
std::string fixed_frame = "left_os1/os1_lidar";

float min_x = 0; 
float max_x = 30;
float min_y = -10;
float max_y = 10;
float min_z = -3;
float max_z = -1;

int channels = 64;

float interval = 0.1800;

int curb_points = 5;
float curb_height = 0.0500;
float angle_filter1 = 150;
float angle_filter2 = 140;

float beam_zone = 30;

int ghost_marker = 0;

ros::Publisher pub_frame;
ros::Publisher pub_non_road;
ros::Publisher pub_road;
ros::Publisher pub_marker_array;

 
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
        int count = 1;
        float epsilon = 0.1;
        int new_end_index = 4;
        
        for (i = 0; i < 4; i++)
        {
            simp_marker_array_points[0][i] = marker_array_points[0][i];
        }

        for (i = new_end_index; i < c - 1; i + 4)
        {
            for (j = 1; j <= 3; j++)
            {
                float d = perpendicular_distance(marker_array_points[i - 4][0], marker_array_points[i - 4][1], 
                                                 marker_array_points[i][0], marker_array_points[i][1],
                                                 marker_array_points[i - j][0], marker_array_points[i - j][1]);
                if (d > epsilon)
                {
                    new_end_index = i - j;
                   
                    for (j = 2; j <= 3; j++)
                    {
                        d = perpendicular_distance(marker_array_points[i - 4][0], marker_array_points[i - 4][1], 
                                                   marker_array_points[new_end_index][0], marker_array_points[new_end_index][1],
                                                   marker_array_points[i - j][0], marker_array_points[i - j][1]);
                    if (d > epsilon)
                    {
                        new_end_index = i - j;
                   
                        for (j = 3; j <= 3; j++)
                        {
                            d = perpendicular_distance(marker_array_points[i - 4][0], marker_array_points[i - 4][1], 
                                                       marker_array_points[new_end_index][0], marker_array_points[new_end_index][1],
                                                       marker_array_points[i - j][0], marker_array_points[i - j][1]);
                        if (d > epsilon)
                        {
                            new_end_index = i - j;
                        }
                        else
                        {

                        }
                    }
                    else
                    {

                    }    
                    // ezt a pontot hozzá kell adni, mert kulcs
                    simp_marker_array_points[count][0] = marker_array_points[i - j][0];             
                    simp_marker_array_points[count][1] = marker_array_points[i - j][1];
                    simp_marker_array_points[count][2] = marker_array_points[i - j][2];
                    simp_marker_array_points[count][3] = marker_array_points[i - j][3];
                    count++;
                }
                else
                {
                    simp_marker_array_points[count][0] = marker_array_points[i][0];             
                    simp_marker_array_points[count][1] = marker_array_points[i][1];
                    simp_marker_array_points[count][2] = marker_array_points[i][2];
                    simp_marker_array_points[count][3] = marker_array_points[i][3];
                    count++;
                }  
            }
        }
            
        for (i = 0; i < 4; i++)
        {
            simp_marker_array_points[count][i] = marker_array_points[c][i];
        }

        std::cout << count << std::endl;        

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

                line_segment.lifetime = ros::Duration(0);
            }

            line_segment.action = visualization_msgs::Marker::DELETE;

            for (int del = line_segment_id; del < ghost_marker; del++)
            {
                line_segment.id++;
                marker_arr.markers.push_back(line_segment);
            }

            ghost_marker = line_segment_id;

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

    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe(topic_name, 1, filter);
    
    pub_frame = node.advertise<pcl::PCLPointCloud2>("filtered_frame", 1);
    pub_non_road = node.advertise<pcl::PCLPointCloud2>("filtered_non_road", 1);
    pub_road = node.advertise<pcl::PCLPointCloud2>("filtered_road", 1);
    pub_marker_array = node.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

    ros::spin();

    return 0;
}