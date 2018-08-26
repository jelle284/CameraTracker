#include "stdafx.h"
#include "math.h"

float LPfilt(float x, float dt, float fc, float y_prev)
{
	float a = 2 * pi*dt*fc / (2 * pi*dt*fc + 1);
	float y = a*x + (1 - a)*y_prev;
	return y;
}

bool intersect(vect &head, lin l1, lin l2)
{
	vect p13, p21, p43, pa, pb;
	float d1343, d4321, d1321, d4343, d2121, den, num, mua, mub;
	float thresh = 0.001;

	p13.x = l1.pa.x - l2.pa.x;
	p13.y = l1.pa.y - l2.pa.y;
	p13.z = l1.pa.z - l2.pa.z;

	p43.x = l2.pb.x - l2.pa.x;
	p43.y = l2.pb.y - l2.pa.y;
	p43.z = l2.pb.z - l2.pa.z;

	//if (fabs(p43.x) < thresh & fabs(p43.y) < thresh & fabs(p43.z) < thresh)
	//{
	//	std::cout << "Intersect error 1" << std::endl;
	//	return false; // error check
	//}

	p21.x = l1.pb.x - l1.pa.x;
	p21.y = l1.pb.y - l1.pa.y;
	p21.z = l1.pb.z - l1.pa.z;

	//if (fabs(p21.x) < thresh & fabs(p21.y) < thresh & fabs(p21.z) < thresh)
	//{
	//	std::cout << "Intersect error 2" << std::endl;
	//	return false; // error check
	//}

	d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
	d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
	d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
	d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
	d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

	den = d2121 * d4343 - d4321 * d4321;

	//if (fabs(den) < thresh)
	//{
	//	std::cout << "Intersect error 3" << std::endl;
	//	return false; // error check
	//}

	num = d1343 * d4321 - d1321 * d4343;

	mua = num / den;
	mub = (d1343 + d4321 * mua) / d4343;

	pa.x = l1.pa.x + mua * p21.x;
	pa.y = l1.pa.y + mua * p21.y;
	pa.z = l1.pa.y + mua * p21.z;

	pb.x = l2.pa.x + mub * p43.x;
	pb.y = l2.pa.y + mub * p43.y;
	pb.z = l2.pa.z + mub * p43.z;

	head.x = (pa.x + pb.x) / 2;
	head.y = (pa.y + pb.y) / 2;
	head.z = (pa.z + pb.z) / 2;

	return true;
}

quat qoffset(quat q, quat t) {
	// offsets q by t

	// t inverse
	quat t_inv, q_out;
	float den = t.w*t.w + t.x*t.x + t.y*t.y + t.z*t.z;
	t_inv.w = t.w / den;
	t_inv.x = -t.x / den;
	t_inv.y = -t.y / den;
	t_inv.z = -t.z / den;

	// multiply q with t
	//q_out.w = q.w*t_inv.w - q.x*t_inv.x - q.y*t_inv.y - q.z*t_inv.z;
	//q_out.x = q.x*t_inv.w + q.w*t_inv.x - q.z*t_inv.y + q.y*t_inv.z;
	//q_out.y = q.y*t_inv.w + q.z*t_inv.x + q.w*t_inv.y - q.x*t_inv.z;
	//q_out.z = q.z*t_inv.w - q.y*t_inv.x + q.x*t_inv.y + q.w*t_inv.z;

	// multiply t with q
	q_out.w = q.w*t_inv.w - q.x*t_inv.x - q.y*t_inv.y - q.z*t_inv.z;
	q_out.x = q.w*t_inv.x + q.x*t_inv.w - q.y*t_inv.z + q.z*t_inv.y;
	q_out.y = q.w*t_inv.y + q.x*t_inv.z + q.y*t_inv.w - q.z*t_inv.x;
	q_out.z = q.w*t_inv.z - q.x*t_inv.y + q.y*t_inv.x + q.z*t_inv.w;

	return q_out;
}

vect::vect(float a, float b, float c) {
	x = a;
	y = b;
	z = c;
}
vect::vect() {
}

vect vect::operator+(vect& v2)
{
	vect v1;
	v1.x = this->x + v2.x;
	v1.y = this->y + v2.y;
	v1.z = this->z + v2.z;
	return v1;
}

vect vect::operator-(vect & v2)
{
	vect v1;
	v1.x = this->x - v2.x;
	v1.y = this->y - v2.y;
	v1.z = this->z - v2.z;
	return v1;
}

quat::quat(float w, float x, float y, float z)
{
	this->w = w;
	this->x = x;
	this->y = y;
	this->z = z;
}

quat::quat()
{
}

lin::lin() {
}

lin::lin(vect pa, vect pb) {
	this->pa = pa;
	this->pb = pb;
}