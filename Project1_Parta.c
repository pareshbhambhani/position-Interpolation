/*
 * Project1_Parta.c
 *
 *  Created on: Mar 15, 2014
 *      Author: PB
 */

#include<stdio.h>
#include<conio.h>
#include<stdlib.h>
#include<stddef.h>


//////Function for Hermite Interpolation//////
float hermite_interpolation(float d1,float d2,float q1,float q2,float t)
{
	float p0,p1,p2,p3,h;
	p0=(2*t*t*t)-(3*t*t)+1;
	p1 = -(2*t*t*t)+(3*t*t);
	p2 = (t*t*t)-(2*t*t)+t;
	p3 = (t*t*t)-(t*t);
	h = p0*q1 + p1*q2 + p2*d1 + p3*d2;
	return h;
}

//////Main begins//////
int main()
{
FILE *fp1, *fp2;
int i=0,j=0,k=0,l=0,frame=1,n1,n2,counter1=0,counter2,noj;
float float_n1,float_n2,t=0,du,q1,q2,d1,d2,h,no_char;

//////Read key and total number of frames and count number of joints//////

fp1 = fopen("robot.KEY","r");
if(fp1==NULL)
{
	printf("Error opening the file\n");
	return (1);
}
else
{
	printf("File opened successfully\n");
}

while((counter2=fscanf(fp1,"%f",&no_char))!=EOF){
	counter1+=counter2;
	}
counter1=counter1-2;
rewind(fp1);
fscanf(fp1,"%d %d",&n1,&n2);
noj=(counter1/(n1*2)); 		/*this gives the number of joints*/
printf("There are %d key frames provided\nTotal number of frames are %d\nThe number of joints in your bot are %d\n",n1,n2,noj);

//////To read positions and velocities from robot.key//////

float q[n1][noj];
float d[n1][noj];
float q_final[n2-1][noj];
for(i=0;i<n1;i++)
{
	for(j=0;j<noj;j++)
	fscanf(fp1,"%f", &q[i][j]);
	for(j=0;j<noj;j++)
	fscanf(fp1,"%f ", &d[i][j]);
}

//////Interpolate//////

float_n1=n1;
float_n2=n2;
du=(float_n1-1)/(float_n2-1);
frame=1;
j=0;
float temp_u=0;
		for(j=0;j<noj;j++)
		{
		k=0;
		for(i=0;i<(n1-1);i++)
		{
		while(t<frame)
		{
		q1=q[i][j];
		q2=q[i+1][j];
		d1=d[i][j];
		d2=d[i+1][j];
		h=hermite_interpolation(d1,d2,q1,q2,temp_u);
		q_final[k][l]=h;
		t=t+du;
		temp_u=temp_u+du;
		k++;
		}
		if(frame <= (n1 - 1))
		{
		frame++;
		temp_u=0;
		}
		else
		break;
		}
		t=0;
		frame=1;
		l++;
		}
		
//////Open file for writing//////

fp2 = fopen("robot.ang","w");
fprintf(fp2,"%d\n",n2);
for(i=0;i<=(n2-1);i++)
{
	j=0;
	while(j<noj)
		{
			fprintf(fp2,"%f ",q_final[i][j]);
			j++;
		}
		fprintf(fp2,"\n");
}

//////Close and exit//////
printf("Your robot.ang file is ready for use\nProgram completed");
fclose(fp1);
fclose(fp2);
return 0;
}
