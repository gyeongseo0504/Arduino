#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

int main(void)
{
    int Input_Current_Angle;
    int Input_Target_Angle;
    int Error;
    int Corrected_Error;

    printf("Input Current Angle = ");
    scanf("%d", &Input_Current_Angle);

    printf("Input Target Angle = ");
    scanf("%d", &Input_Target_Angle);

    Error = Input_Target_Angle - Input_Current_Angle;

    printf("o.Error  : %d = %d - %d \n", Error, Input_Target_Angle, Input_Current_Angle);

    printf("c.Error  : %d = %d - %d \n", Corrected_Error, Input_Target_Angle, Input_Current_Angle);

    if (Error < 180 && Error >0)
    {
        Corrected_Error = Input_Target_Angle - Input_Current_Angle;
        printf("c.Error  : %d  =  %d  -  %d", Corrected_Error, Input_Target_Angle, Input_Current_Angle);
    }
    else if (Error > 180)
    {
        Corrected_Error = Error - 360;
        printf("c.Error  : %d  =  %d  -  %d", Corrected_Error, Input_Target_Angle, Input_Current_Angle);
    }
    else if (Error < 0 && Error > -180)
    {
        Corrected_Error = Error;
        printf("c.Error  : %d  =  %d  -  %d", Corrected_Error, Input_Target_Angle, Input_Current_Angle);
    }
    else if (Error < -180 && Error > -360)
    {
        Corrected_Error = 360 + Error;
        printf("c.Error  : %d  =  %d  -  %d", Corrected_Error, Input_Target_Angle, Input_Current_Angle);
    }
    else if (Error == 0 || Error == 360 || Error == -360)
    {
        Corrected_Error = 0;
        printf("c.Error  : %d  =  %d  -  %d", Corrected_Error, Input_Target_Angle, Input_Current_Angle);
    }
    else if (Error == 180 || Error == -180)
    {
        Corrected_Error = 180;
        printf("c.Error  : %d  =  %d  -  %d", Corrected_Error, Input_Target_Angle, Input_Current_Angle);
    }
}
