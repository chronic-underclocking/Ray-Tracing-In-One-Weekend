#pragma once

#include "hittable.h"

class sphere : public hittable {
  public:
    sphere(const point3& center, double radius, shared_ptr<material> mat)
      : center(center), radius(std::fmax(0,radius)), mat(mat) {}

    // (Cx−X)^2 + (Cy−Y)^2 + (Cz−Z)^2 = r^2
    // (C−P)⋅(C−P) = (Cx−X)^2 + (Cy−Y)^2 + (Cz−Z)^2
    // (C−P)⋅(C−P) = r^2
    // P(t) = Q + td
    // (C−P(t))⋅(C−P(t)) = r^2
    // (C−(Q+td))⋅(C−(Q+td)) = r^2
    // (−td+(C−Q))⋅(−td+(C−Q)) = r^2
    // (t^2)d⋅d − 2td⋅(C−Q) + (C−Q)⋅(C−Q) = r^2
    // (t^2)d⋅d − 2td⋅(C−Q) + (C−Q)⋅(C−Q) − r^2 = 0
    // at^2 + bt + c = 0,
    // (−b ± √( b^2 − 4ac )) / 2a
    //
    // a = d⋅d
    // b = −2d⋅(C−Q)
    // c = (C−Q)⋅(C−Q) − r^2
    //
    // b = −2h
    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        vec3 oc = center - r.origin();
        auto a = r.direction().length_squared();
        auto h = dot(r.direction(), oc);
        auto c = oc.length_squared() - radius*radius;

        auto discriminant = h*h - a*c;
        if (discriminant < 0)
            return false;

        auto sqrtd = std::sqrt(discriminant);

        // Find the nearest root that lies in the acceptable range.
        auto root = (h - sqrtd) / a;
        if (!ray_t.surrounds(root)) {
            root = (h + sqrtd) / a;
            if (!ray_t.surrounds(root))
                return false;
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        vec3 outward_normal = (rec.p - center) / radius;
        rec.set_face_normal(r, outward_normal);
        rec.mat = mat;

        return true;
    }

  private:
    point3 center;
    double radius;
    shared_ptr<material> mat;
};