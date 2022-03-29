#include <stdio.h>
#include <math.h>

float datax[] = {};
float datay[] = {};
float n =0;

float dis(float x, float y, float a, float b)
{
    return abs(a * x - y + b) / sqrt(a * a + b * b); //점과 선의 거리 = 오차
}

float fxy(float a, float b)
{
    float sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        sum += dis(datax[i], datay[i], a, b);
    }
    return sum / n;
}

float dfxydx(float x, float y, float dx) //x 편미분
{
    return (fxy(x + dx, y) - fxy(x, y)) / dx;
}

float dfxydy(float x, float y, float dy)  //y 편미분
{
    return (fxy(x, y-dy) - fxy(x, y)) / dy;
}

int main()
{
    
    for (int i = 0; i <= 20; i++)
    {
        datax[i] = i-0.5;
        datay[i] = i+0.5;
        n += i;
        printf("%f %f\n", datax[i], datay[i]);
    }

    float dx = 0.01;
    float dy = 0.01;
    float learing_rate = 0.1;
    float a, a1 = 1; //임의로 찍어주는 기울기
    float b, b1 = 1;

    for(int i = 0; i<500; i++) {
        a = a1;
        b = b1;
        a1 -= learing_rate * dfxydx(a,b,dx);
        b1 -= learing_rate * dfxydy(a,b,dy);
    }

    printf("y = %fx + %f", a1, b1);

    return 0;
}