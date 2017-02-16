/*
 * partb.c
 *
 *  Created on: Mar 22, 2014
 *      Author: PB
 */
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<stddef.h>

#define MAX_CONFIG  100
#define SIZE 1500

float s,step_size;
int n1,n2;
float ARR_BEZIER[SIZE][4];
float ARR_ROTATION[SIZE][3];

struct vector {
			float x;
			float y;
			float z;
			};
struct quaternion {
	struct vector v;
	float s;
	};
struct quaternion q[MAX_CONFIG];

float dot(v1, v2)
struct vector v1,v2;
{
	float d;

	d = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;;

	return(d);
}

struct vector cross(v1, v2)
struct vector v1,v2;
{
	struct vector v;

	v.x = v1.y * v2.z - v1.z * v2.y;
	v.y = v1.z * v2.x - v1.x * v2.z;
	v.z = v1.x * v2.y - v1.y * v2.x;

	return(v);
}

float Qdot(q1, q2)
struct quaternion q1,q2;
{
	float d;

	d = q1.v.x * q2.v.x + q1.v.y * q2.v.y +
	    q1.v.z * q2.v.z + q1.s   * q2.s;

	return(d);
}

struct quaternion Qadd(q1, q2)
struct quaternion q1,q2;
{
	struct quaternion q;

	q.v.x = q1.v.x + q2.v.x;
	q.v.y = q1.v.y + q2.v.y;
	q.v.z = q1.v.z + q2.v.z;
	q.s   = q1.s   + q2.s;

	return(q);
}

struct quaternion Qmult(q1, q2)
struct quaternion q1,q2;
{
	struct quaternion q;
	struct vector cross();

	q.v = cross(q1.v,q2.v);
	q.v.x = q.v.x + q2.s*q1.v.x + q1.s*q2.v.x;
	q.v.y = q.v.y + q2.s*q1.v.y + q1.s*q2.v.y;
	q.v.z = q.v.z + q2.s*q1.v.z + q1.s*q2.v.z;
	q.s   = q1.s * q2.s - dot(q1.v,q2.v);

	return(q);
}

float Qnorm2(q)
struct quaternion q;
{
	float d,dot();

	d = q.s * q.s + dot(q.v,q.v);

	return(d);
}

struct quaternion Qinv(q)
struct quaternion q;
{
	struct quaternion qinv;
	float d,Qnorm2();

	d = Qnorm2(q);

	qinv.v.x = -q.v.x/d;
	qinv.v.y = -q.v.y/d;
	qinv.v.z = -q.v.z/d;
	qinv.s   = q.s/d;

	return(qinv);
}

struct quaternion Double(q1,q2)
struct quaternion q1,q2;
{
	struct quaternion q;
	float d2,Qdot();

	d2 = 2 * Qdot(q1,q2);

	q.v.x = d2 * q2.v.x - q1.v.x;
	q.v.y = d2 * q2.v.y - q1.v.y;
	q.v.z = d2 * q2.v.z - q1.v.z;
	q.s   = d2 * q2.s   - q1.s;

	return(q);
}

struct quaternion Bisect(q1,q2)
struct quaternion q1,q2;
{
	struct quaternion q, Qadd();
	float d, Qnorm2();

	q = Qadd(q1,q2);
	d = sqrt(Qnorm2(q));

	q.v.x = q.v.x/d;
	q.v.y = q.v.y/d;
	q.v.z = q.v.z/d;
	q.s   = q.s/d;

	return(q);
}

struct quaternion slerp(q1,q2,u)
struct quaternion q1,q2;
float u;
{
	struct quaternion qu;
	float sine, cosine, theta, a1, a2, Qdot();

	cosine = Qdot(q1,q2);
	sine = sqrt(1 - cosine*cosine);
	theta = atan2(sine,cosine);

	if (sine < 0.0001)
	     {
		a1 = 1-u;
		a2 = u;
	     }
	else {
		a1 = sin( (1-u)*theta )/sine;
		a2 = sin( u*theta )/sine;
	     }

	qu.v.x = a1 * q1.v.x + a2 * q2.v.x;
	qu.v.y = a1 * q1.v.y + a2 * q2.v.y;
	qu.v.z = a1 * q1.v.z + a2 * q2.v.z;
	qu.s   = a1 * q1.s   + a2 * q2.s;

	return(qu);
}



//////Q to R//////
struct vector Quat_Rotation()
{
	struct vector n, o, a, v;
	float s1;
	int j,size_arr=0;
	for(j=0;j<n2;j++)
	{
	v.x = ARR_BEZIER[j][0];
	v.y = ARR_BEZIER[j][1];
	v.z = ARR_BEZIER[j][2];
	s1 = ARR_BEZIER[j][3];
	n.x = 1 - 2 * (v.y*v.y + v.z*v.z);
	n.y = 2 * (v.x*v.y + s1*v.z);
	n.z = 2 * (v.x*v.z - s1*v.y);

	o.x = 2 * (v.x*v.y - s1*v.z);
	o.y = 1 - 2 * (v.x*v.x + v.z*v.z);
	o.z = 2 * (v.y*v.z + s1*v.x);

	a.x = 2 * (v.x*v.z + s1*v.y);
	a.y = 2 * (v.y*v.z - s1*v.x);
	a.z = 1 - 2 * (v.x*v.x + v.y*v.y);
	ARR_ROTATION[size_arr][0]=n.x;
	ARR_ROTATION[size_arr][1]=o.x;
	ARR_ROTATION[size_arr][2]=a.x;
	ARR_ROTATION[size_arr+1][0]=n.y;
	ARR_ROTATION[size_arr+1][1]=o.y;
	ARR_ROTATION[size_arr+1][2]=a.y;
	ARR_ROTATION[size_arr+2][0]=n.z;
	ARR_ROTATION[size_arr+2][1]=o.z;
	ARR_ROTATION[size_arr+2][2]=a.z;
	size_arr=size_arr+3;
	}
}

//////R to Q//////

struct vector quaternion(float nx,float ox, float ax,float ny,float oy, float ay,float nz,float oz, float az)
{
		struct vector v;
		float dx, dy, dz, ds;
		dx = 1 + nx - oy - az;
		dy = 1 - nx + oy - az;
		dz = 1 - nx - oy + az;
		ds = 1 + nx + oy + az;


		switch( max(dx, dy, dz, ds) ) {
		case 1 : v.x = sqrt(dx)/2;
			 v.y = (ny + ox)/(4 * v.x);
			 v.z = (nz + ax)/(4 * v.x);
			 s   = (oz - ay)/(4 * v.x);
		break;

		case 2 : v.y = sqrt(dy)/2;
			 v.x = (ny + ox)/(4 * v.y);
			 v.z = (oz + ay)/(4 * v.y);
			 s   = (ax - nz)/(4 * v.y);
		break;

		case 3 : v.z = sqrt(dz)/2;
			 v.x = (nz + ax)/(4 * v.z);
			 v.y = (oz + ay)/(4 * v.z);
			 s   = (ny - ox)/(4 * v.z);
		break;

		case 4 : s   = sqrt(ds)/2;
			 v.x = (oz - ay)/(4 * s);
			 v.y = (ax - nz)/(4 * s);
			 v.z = (ny - ox)/(4 * s);
		}

		return v;
	}

	int max(a, b, c, d)
	float a, b, c, d;
	{
		int m;

		if ( a >= b && a >= c && a >= d ) m = 1;
		if ( b >= a && b >= c && b >= d ) m = 2;
		if ( c >= a && c >= b && c >= d ) m = 3;
		if ( d >= a && d >= b && d >= c ) m = 4;

		return(m);
	}


///////BEZIER_INTERPOLATION function///////

struct quaternion BEZIER_INTERPOLATION (float ARR_QUATERNION[][4])
{

	struct quaternion a[MAX_CONFIG], b[MAX_CONFIG],qu,
				  Bisect(), Double(), slerp();
		float u;
		float float_n1,float_n2;
		int numb_config,i,j;
		numb_config = n1;
		for (i=0; i<numb_config; ++i)
		   {
			q[i].v.x=ARR_QUATERNION[i][0];
			q[i].v.y=ARR_QUATERNION[i][1];
			q[i].v.z=ARR_QUATERNION[i][2];
			q[i].s=ARR_QUATERNION[i][3];
		   }
		float_n1=n1;
		float_n2=n2;
		step_size=(float_n1-1)/(float_n2-1);
		i=0;

		/* compute intermediate configuration */
		//////For q0 and q1//////
		if(i==0)
		{
			a[i]=q[i];
			a[i] = slerp ( q[i], a[i], 1.0/3.0 );
		}
		//////For q1 to qn-1//////
		for (i=1; i < (numb_config-1); ++i)
		   {
			a[i] = Bisect( Double( q[i-1], q[i] ), q[i+1] );
			a[i] = slerp ( q[i], a[i], 1.0/3.0 );
			b[i] = Double( a[i], q[i] );
		   }
		//////for qn-1 to qn//////
		if(i==numb_config-1)
		{
			b[i]=q[i];
		}
		j=0;
		for (i=0; i < (numb_config-1); ++i)
		   {
			for (u=0; u < 1; u = u + step_size)
			    {
				qu = slerp( a[i], b[i+1], u);
				qu = slerp(slerp( slerp( q[i], a[i], u ), qu,    u),
				           slerp( qu, slerp( b[i+1], q[i+1], u), u),
					   u);
				ARR_BEZIER[j][0]=qu.v.x;
				ARR_BEZIER[j][1]=qu.v.y;
				ARR_BEZIER[j][2]=qu.v.z;
				ARR_BEZIER[j][3]=qu.s;
				j++;
			    }
		   }
}

//////Catmull ROM//////
float CATMULL_ROM(float pos0,float pos1,float pos2,float pos3,float step_size)
{
	float Pos0,Pos1,Pos2,Pos3,Position_Interpolated;
	Pos0 = (-0.5*step_size*step_size*step_size)+(step_size*step_size)-(0.5*step_size);
	Pos1 = (1.5*step_size*step_size*step_size)-(2.5*step_size*step_size)+1;
	Pos2 = (-1.5*step_size*step_size*step_size)+(2*step_size*step_size)+(0.5*step_size);
	Pos3 = (0.5*step_size*step_size*step_size)-(0.5*step_size*step_size);
	Position_Interpolated = Pos0*pos0 + Pos1*pos1 + Pos2*pos2 + Pos3*pos3;
	return Position_Interpolated;
}


//////Main Begins//////

int main()
{
struct vector vect;
float normal_x,normal_y,normal_z,orientation_x,orientation_y,orientation_z,approach_x,approach_y,approach_z;
float CR_Point1x,CR_Point2x,CR_Point3x,CR_Point4x;
float CR_Point1y,CR_Point2y,CR_Point3y,CR_Point4y;
float CR_Point1z,CR_Point2z,CR_Point3z,CR_Point4z;
float z=0,u=0;
int temp1=0,temp2=0,temp3=0,count=0,frame=1;
FILE *fp1,*fp2;

//////Read key file for values//////
fp1 = fopen("object.KEY","r");
if(fp1==NULL)
{
	printf("Error opening the file");
	return (1);
}
fscanf(fp1,"%d %d",&n1,&n2);
float Arr_Rotation[n1*3][3];
float Arr_Position[n1][3];
float ARR_QUATERNION[n1][4];
float Position_Out[n2][3];

for(temp1=0;temp1<(3*n1);temp1++)
{
	fscanf(fp1,"%f %f %f",&Arr_Rotation[temp1][0],&Arr_Rotation[temp1][1],&Arr_Rotation[temp1][2]);
	fscanf(fp1,"%f",&Arr_Position[temp3][temp2]);
	temp2++;
	if(temp2==3)
	{
		temp2=0;
		temp3++;
	}
}

//////Convert to Quaternion//////
temp1=0;
temp2=0;
while(temp1<(3*n1))
{
	normal_x=Arr_Rotation[temp1][0];
	orientation_x=Arr_Rotation[temp1][1];
	approach_x=Arr_Rotation[temp1][2];
	normal_y=Arr_Rotation[temp1+1][0];
	orientation_y=Arr_Rotation[temp1+1][1];
	approach_y=Arr_Rotation[temp1+1][2];
	normal_z=Arr_Rotation[temp1+2][0];
	orientation_z=Arr_Rotation[temp1+2][1];
	approach_z=Arr_Rotation[temp1+2][2];
	vect=quaternion(normal_x,orientation_x,approach_x,normal_y,orientation_y,approach_y,normal_z,orientation_z,approach_z);
	ARR_QUATERNION[temp2][0]=vect.x;
	ARR_QUATERNION[temp2][1]=vect.y;
	ARR_QUATERNION[temp2][2]=vect.z;
	ARR_QUATERNION[temp2][3]=s;
	temp2++;
	temp1=temp1+3;
}

//////Bezier Interpolation//////
BEZIER_INTERPOLATION(ARR_QUATERNION);

//////Quaternion to Rotation conversion//////
Quat_Rotation();

///// Positional interpolation//////

temp3=0;
for(temp2=0;temp2<=(n1-2);temp2++)
{
	while(z<frame)
	{
	if(temp2==0)
	{
		CR_Point2x=Arr_Position[temp2][temp3];
		CR_Point2y=Arr_Position[temp2][temp3+1];
		CR_Point2z=Arr_Position[temp2][temp3+2];
		CR_Point1x=Arr_Position[temp2+1][temp3];
		CR_Point1y=Arr_Position[temp2+1][temp3+1];
		CR_Point1z=Arr_Position[temp2+1][temp3+2];
		CR_Point3x=CR_Point1x;
		CR_Point3y=CR_Point1y;
		CR_Point3z=CR_Point1z;
		CR_Point4x=Arr_Position[temp2+2][temp3];
		CR_Point4y=Arr_Position[temp2+2][temp3+1];
		CR_Point4z=Arr_Position[temp2+2][temp3+2];
	}
	else if(temp2>0 && temp2<n1-2)
	{
		CR_Point1x=Arr_Position[temp2-1][temp3];
		CR_Point1y=Arr_Position[temp2-1][temp3+1];
		CR_Point1z=Arr_Position[temp2-1][temp3+2];
		CR_Point2x=Arr_Position[temp2][temp3];
		CR_Point2y=Arr_Position[temp2][temp3+1];
		CR_Point2z=Arr_Position[temp2][temp3+2];
		CR_Point3x=Arr_Position[temp2+1][temp3];
		CR_Point3y=Arr_Position[temp2+1][temp3+1];
		CR_Point3z=Arr_Position[temp2+1][temp3+2];
		CR_Point4x=Arr_Position[temp2+2][temp3];
		CR_Point4y=Arr_Position[temp2+2][temp3+1];
		CR_Point4z=Arr_Position[temp2+2][temp3+2];
	}
	else if(temp2==n1-2)
	{
		CR_Point1x=Arr_Position[temp2-1][temp3];
		CR_Point1y=Arr_Position[temp2-1][temp3+1];
		CR_Point1z=Arr_Position[temp2-1][temp3+2];
		CR_Point2x=Arr_Position[temp2][temp3];
		CR_Point2y=Arr_Position[temp2][temp3+1];
		CR_Point2z=Arr_Position[temp2][temp3+2];
		CR_Point3x=Arr_Position[temp2+1][temp3];
		CR_Point3y=Arr_Position[temp2+1][temp3+1];
		CR_Point3z=Arr_Position[temp2+1][temp3+2];
		CR_Point4x=CR_Point2x;
		CR_Point4y=CR_Point2y;
		CR_Point4z=CR_Point2z;
	}
	Position_Out[count][0]=CATMULL_ROM(CR_Point1x,CR_Point2x,CR_Point3x,CR_Point4x,u);
	Position_Out[count][1]=CATMULL_ROM(CR_Point1y,CR_Point2y,CR_Point3y,CR_Point4y,u);
	Position_Out[count][2]=CATMULL_ROM(CR_Point1z,CR_Point2z,CR_Point3z,CR_Point4z,u);
	z=z+step_size;
	u=u+step_size;
	count++;
	}
	if(frame <= (n1 - 1))
	{
	frame++;
	u=0;
	}
	else
		break;
}

//////Write to the traj file//////
fp2 = fopen("object.traj","w");
fprintf(fp2,"%d\n",n2);
temp2=0;
temp3=0;
	for(temp1=0;temp1<3*(n2);temp1++)
	{
		fprintf(fp2,"%f %f %f %f\n",ARR_ROTATION[temp1][0], ARR_ROTATION[temp1][1],ARR_ROTATION[temp1][2], Position_Out[temp3][temp2]);
		temp2++;
		if(temp2==3)
		{temp3++;
		temp2=0;}
}

fclose(fp1);
fclose(fp2);
printf("The trajectory file is ready\nProgram complete");
return 0;
}
