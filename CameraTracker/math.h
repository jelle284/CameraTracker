#pragma once

const float pi = 3.14159265359;

struct vect {
	vect();
	vect(float a, float b, float c);
	float x, y, z;
	vect operator+(vect& v2);
	vect operator-(vect& v2);
};

struct quat {
	quat(float w, float x, float y, float z);
	quat();
	float w, x, y, z;
};

struct lin {
	lin();
	lin(vect pa, vect pb);
	vect pa, pb;
};

bool intersect(vect &head, lin l1, lin l2);

float LPfilt(float x, float dt, float fc, float y_prev); // RC low pass filter

quat qoffset(quat q, quat t);

