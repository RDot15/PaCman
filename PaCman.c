//��#   P a C m a n 
 
#include <stdio.h>
#include <stdlib.h>
#include <conio.h>
#include <time.h>
#include <locale.h>

#define TEN 10
#define TWENTY_ONE 21

typedef unsigned short u16;
typedef unsigned char u8;

void print_array(char array[][TWENTY_ONE], u8 size){
    for (u8 i = 0; i < size; i++)
        printf("%s\n", array[i]);
}

int main(int argc, char const *argv[])
{
    char massiv[TEN][TWENTY_ONE];
    char key;
    u8 xpos = 10;
    u8 ypos = 5;
    u8 before_xpos;
    u8 before_ypos;
    u8 bait_x = 4, bait_y = 3;
    u16 count = 0;
    srand(time(NULL));
    
    do
    {
        sprintf(massiv[0], "####################");
        for(u8 i = 1; i < 9; i ++)
            sprintf(massiv[i], "#                  #");
        sprintf(massiv[9], "####################");
        
        massiv[ypos][xpos] = '@';
        massiv[bait_y][bait_x]  = '*';   

        system("cls");
    
        print_array(massiv, TEN);
        key = getch();

        before_xpos = xpos;
        before_ypos = ypos;

        switch (key)
        {
            case 'w':
                ypos --; 
            break;
        
            case 's': 
                ypos ++;
            break;
        
            case 'a':
                xpos --;
            break;
        
            case 'd':
                xpos ++;
            break;
        
            default:
            break;
        }

        if (massiv[ypos][xpos] == '#')
        {
            xpos = before_xpos;
            ypos = before_ypos;
        }

        if ((xpos == bait_x) && (ypos == bait_y))
        {
            bait_x = rand() % 18 + 1;
            bait_y = rand() % 8 + 1;
            count ++;
        }    


    } while (key != 't');
    
    printf ("\nYour count  %d ", count);
    return 0;
}
