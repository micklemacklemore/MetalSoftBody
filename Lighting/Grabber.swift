

class Grabber {
    struct Ray {
        var orig: SIMD3<Float>
        var dir: SIMD3<Float>
    }
    
    let renderer: Renderer!
    
    init (renderer: Renderer) {
        self.renderer = renderer
    }
    
    func rayCast (mousex x: Float, mousey y: Float) -> Bool {
        // screen "bounds" coordinates to clip space
        let x: Float = (2.0 * x) / Float(renderer.viewBounds.width) - 1.0;
        let y: Float = 1.0 - (2.0 * y) / Float(renderer.viewBounds.height);
        let z: Float = 1.0;
        let ray_clip: SIMD4<Float> = SIMD4<Float>(x, y, z, 1.0)
        
        // clip space to eye space
        var ray_eye: SIMD4<Float> = renderer.camera.projectionMatrix.inverse * ray_clip
        ray_eye.z = 1.0
        ray_eye.w = 0.0
        
        // eye space to world space
        var ray_world: SIMD3<Float> = (renderer.camera.viewMatrix.inverse * ray_eye).xyz
        ray_world = normalize(ray_world)
        
        let ray = Ray(orig: renderer.camera.position, dir: ray_world)
        
        let bboxresult = intersectAABB(ray: ray, min: renderer.softbody.boundingBox.min, max: renderer.softbody.boundingBox.max)
        print("bbox hit? \(bboxresult)")
        
        if (bboxresult) {
            
            
            return true
        }
        
        return false
    }
    
    func intersectAABB(ray r: Ray, min: SIMD3<Float>, max: SIMD3<Float>) -> Bool {
        var tmin: Float = (min.x - r.orig.x) / r.dir.x
        var tmax: Float = (max.x - r.orig.x) / r.dir.x
        
        if (tmin > tmax) {
            swap(&tmin, &tmax)
        }
        
        var tymin: Float = (min.y - r.orig.y) / r.dir.y
        var tymax: Float = (max.y - r.orig.y) / r.dir.y
        
        if (tymin > tymax) {
            swap(&tymin, &tymax)
        }
        
        if ((tmin > tymax) || (tymin > tmax)) {
            return false
        }
        
        if (tymin > tmin) {
            tmin = tymin
        }
        if (tymax < tmax) {
            tmax = tymax
        }
        
        var tzmin: Float = (min.z - r.orig.z) / r.dir.z
        var tzmax: Float = (max.z - r.orig.z) / r.dir.z
        
        if (tzmin > tzmax) {
            swap(&tzmin, &tzmax)
        }
        
        if ((tmin > tzmax) || (tzmin > tmax)) {
            return false
        }
        
        if (tzmin > tmin) {
            tmin = tzmin
        }
        if (tzmax < tmax) {
            tmax = tzmax
        }
        
        return true
    }
}

//bool rayTriangleIntersect(
//    const Vec3f &orig, const Vec3f &dir,
//    const Vec3f &v0, const Vec3f &v1, const Vec3f &v2,
//    float &t, float &u, float &v)
//{
//    // compute plane's normal
//    Vec3f v0v1 = v1 - v0;
//    Vec3f v0v2 = v2 - v0;
//    // no need to normalize
//    Vec3f N = v0v1.crossProduct(v0v2); // N
//    float denom = N.dotProduct(N);
//
//    // Step 1: finding P
//
//    // check if ray and plane are parallel ?
//    float NdotRayDirection = N.dotProduct(dir);
//
//    if (fabs(NdotRayDirection) < kEpsilon) // almost 0
//        return false; // they are parallel so they don't intersect !
//
//    // compute d parameter using equation 2
//    float d = -N.dotProduct(v0);
//
//    // compute t (equation 3)
//    t = -(N.dotProduct(orig) + d) / NdotRayDirection;
//
//    // check if the triangle is in behind the ray
//    if (t < 0) return false; // the triangle is behind
//
//    // compute the intersection point using equation 1
//    Vec3f P = orig + t * dir;
//
//    // Step 2: inside-outside test
//    Vec3f C; // vector perpendicular to triangle's plane
//
//    // Calculate u (for triangle BCP)
//    Vec3f v1p = P - v1;
//    Vec3f v1v2 = v2 - v1;
//    C = v1v2.crossProduct(v1p);
//    if ((u = N.dotProduct(C)) < 0) return false; // P is on the right side
//
//    // Calculate v (for triangle CAP)
//    Vec3f v2p = P - v2;
//    Vec3f v2v0 = v0 - v2;
//    C = v2v0.crossProduct(v2p);
//    if ((v = N.dotProduct(C)) < 0) return false; // P is on the right side
//
//    Vec3f v0p = P - v0;
//    C = v0v1.crossProduct(v0p);
//    if (N.dotProduct(C) < 0) return false; // P is on the right side
//
//    u /= denom;
//    v /= denom;
//
//    return true; // this ray hits the triangle
//}
