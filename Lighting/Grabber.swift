import MetalKit

struct Ray {
    var orig: SIMD3<Float>
    var dir: SIMD3<Float>
}

struct HitResult {
    var intersectPoint: SIMD3<Float>
    var closestParticleIdx: Int
    var distance: Float
}

class Grabber {
    let renderer: Renderer!
    
    var prevPos: SIMD3<Float> = .zero
    var vel: SIMD3<Float> = .zero
    var time: Double = 0.0
    var distance: Float = 0.0
    var physicsObject: SoftBody? = nil
    
    init (renderer: Renderer) {
        self.renderer = renderer
    }
    
    func increaseTime(dt: Double) {
        self.time += dt
    }
    
    func start(mousex x: Float, mousey y: Float) -> Bool {
        let ray = mouseCoordsToWorldSpaceRay(x, y)
        
        let bboxresult = Grabber.intersectAABB(ray: ray, min: renderer.softbody.boundingBox.min, max: renderer.softbody.boundingBox.max)
        
        if bboxresult, let result = getTriangle(ray: ray) {
            let pos = result.intersectPoint
            renderer.debugIntersect = pos
            self.distance = result.distance
            self.prevPos = pos
            self.vel = .zero
            self.time = 0.0
            
            self.physicsObject = renderer.softbody
            self.physicsObject!.startGrab(
                idx: result.closestParticleIdx,
                pos: pos
            )
            
            return true
        }
        
        return false
    }
    
    func move(mousex x: Float, mousey y: Float) {
        let ray = mouseCoordsToWorldSpaceRay(x, y)
        let pos = ray.orig + ray.dir * self.distance
        renderer.debugIntersect = pos
        
        self.vel = pos - self.prevPos
        if (self.time > 0.0) {
            self.vel /= Float(self.time)
        } else {
            self.vel = .zero
        }
        self.prevPos = pos
        self.time = 0.0
        
        self.physicsObject!.updateGrab(pos: pos, vel: self.vel)
    }
    
    func end() {
        if physicsObject != nil {
            self.physicsObject!.endGrab(pos: self.prevPos, vel: self.vel)
            self.physicsObject = nil
        }
    }
    
    func mouseCoordsToWorldSpaceRay(_ x: Float, _ y: Float) -> Ray {
        // screen "bounds" coordinates to clip space
        let ray_clip: SIMD4<Float> = SIMD4<Float>(
            (2.0 * x) / Float(renderer.viewBounds.width) - 1.0,
            1.0 - (2.0 * y) / Float(renderer.viewBounds.height),
            1.0,
            1.0
        )
        
        // clip space to eye space
        var ray_eye: SIMD4<Float> = renderer.camera.projectionMatrix.inverse * ray_clip
        ray_eye.z = 1.0
        ray_eye.w = 0.0
        
        return Ray(orig: renderer.camera.position, dir: normalize((renderer.camera.viewMatrix.inverse * ray_eye).xyz))
    }
    
    func getTriangle(ray: Ray) -> HitResult? {
        let sb = renderer.softbody!
        var mint: Float = .greatestFiniteMagnitude
        var minu: Float = 0.0
        var minv: Float = 0.0
        var trianglePos: [SIMD3<Float>] = []
        var triangleIdx: [Int] = []
        
        var hit: Bool = false
        
        for i in stride(from: 0, to: sb.surfaceIds.count, by: 3) {
            
            let i0 = Int(sb.surfaceIds[i])     * 3
            let i1 = Int(sb.surfaceIds[i + 1]) * 3
            let i2 = Int(sb.surfaceIds[i + 2]) * 3
            
            let v0 = SIMD3<Float>(
                sb.pos[i0],
                sb.pos[i0 + 1],
                sb.pos[i0 + 2]
            )

            let v1 = SIMD3<Float>(
                sb.pos[i1],
                sb.pos[i1 + 1],
                sb.pos[i1 + 2]
            )

            let v2 = SIMD3<Float>(
                sb.pos[i2],
                sb.pos[i2 + 1],
                sb.pos[i2 + 2]
            )
            
            var t: Float = 0.0, u: Float = 0.0, v: Float = 0.0
            
            let result = Grabber.intersectTriangle(ray: ray, v0: v0, v1: v1, v2: v2, t: &t, u: &u, v: &v)
            
            if (!result) {
                continue
            }
            
            hit = true
            
            if (t < mint) {
                mint = t
                minu = u
                minv = v
                trianglePos = [v0, v1, v2]
                triangleIdx = [
                    Int(sb.surfaceIds[i]),
                    Int(sb.surfaceIds[i + 1]),
                    Int(sb.surfaceIds[i + 2])
                ]
            }
        }
        
        if (!hit) {
            return nil
        }
        
        let w: Float = 1.0 - minu - minv
        
        var hitresult: HitResult = HitResult(intersectPoint: .zero, closestParticleIdx: 0, distance: mint)
        
        // closest particle. we return the index used for SoftBody.pos
        
        // u
        if (minu >= minv && minu >= w) {
            hitresult.closestParticleIdx = triangleIdx[0]
        }
        // v
        if (minv >= minu && minv >= w) {
            hitresult.closestParticleIdx = triangleIdx[1]
        }
        // w
        hitresult.closestParticleIdx = triangleIdx[2]
        
        // intersect point
        hitresult.intersectPoint = minu * trianglePos[0] + minv * trianglePos[1] + w * trianglePos[2]
        
        return hitresult
    }
    
    static func intersectAABB(ray r: Ray, min: SIMD3<Float>, max: SIMD3<Float>) -> Bool {
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
    
    static func intersectTriangle(ray: Ray, v0: SIMD3<Float>, v1: SIMD3<Float>, v2: SIMD3<Float>, t: inout Float, u: inout Float, v: inout Float) -> Bool {
        
        let orig = ray.orig
        let dir = ray.dir
        
        // compute plane's normal
        let v0v1 = v1 - v0
        let v0v2 = v2 - v0
        
        let N = cross(v0v1, v0v2)
        let denom = dot(N, N)
        
        // Step 1: finding P
        
        // check if ray and plane are parallel ?
        let NdotRayDirection = dot(N, dir)
        
        if abs(NdotRayDirection) < 0.0001 {
            return false
        }

        let d = -dot(N, v0)
        t = -(dot(N, orig) + d) / NdotRayDirection
        
        if (t < 0) {
            return false
        }
        
        // compute the intersection point
        let P = orig + t * dir
        
        // Step 2: inside-outside test
        var C: SIMD3<Float>
        
        let v1p = P - v1
        let v1v2 = v2 - v1
        C = cross(v1v2, v1p)
        u = dot(N, C)
        if (u < 0) {
            return false
        }
        
        let v2p = P - v2
        let v2v0 = v0 - v2
        C = cross(v2v0, v2p)
        v = dot(N, C)
        if (v < 0) {
            return false
        }
        
        let v0p = P - v0
        C = cross(v0v1, v0p)
        
        if (dot(N, C) < 0) {
            return false
        }
        
        u /= denom
        v /= denom
        
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

