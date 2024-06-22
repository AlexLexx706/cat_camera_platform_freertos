#ifndef _VECTOR_3D_H
#define _VECTOR_3D_H

struct Vector3D {
    float x0;
    float x1;
    float x2;

    float &operator[](const int id) {
        return reinterpret_cast<float *>(this)[id];
    }

    Vector3D &operator=(const Vector3D &v) {
        x0 = v.x0;
        x1 = v.x1;
        x2 = v.x2;
        return *this;
    }

    Vector3D operator+(const Vector3D &v) const {
        return {x0 + v.x0, x1 + v.x1, x2 + v.x2};
    }

    Vector3D operator-(const Vector3D &v) const {
        return {x0 - v.x0, x1 - v.x1, x2 - v.x2};
    }

    Vector3D operator*(const Vector3D &v) const {
        return {x0 * v.x0, x1 * v.x1, x2 * v.x2};
    }

    Vector3D operator*(const float v) const {
        return {x0 * v, x1 * v, x2 * v};
    }

    Vector3D &operator+=(const Vector3D &v) {
        x0 += v.x0;
        x1 += v.x1;
        x2 += v.x2;
        return *this;
    }

    Vector3D &operator*=(const float v) {
        x0 *= v;
        x1 *= v;
        x2 *= v;
        return *this;
    }

    Vector3D &operator*=(const Vector3D & v) {
        x0 *= v.x0;
        x1 *= v.x1;
        x2 *= v.x2;
        return *this;
    }

    bool operator != (const Vector3D & v) {
        return x0 != v.x0 || x1 != v.x1 || x2 != v.x2;
    }

};

#endif //_VECTOR_3D_H