import MetalKit

struct BoundingBox {
    var min: SIMD3<Float>
    var max: SIMD3<Float>
}

class SoftBody: Node {
    // MARK: simulation vars
    
    var numParticles: Int!
    var numTets: Int!
    
    var pos: [Float]!
    var prevPos: [Float]!
    var vel: [Float]!
    
    var normals: [Float]!
    
    var tetIds: [Int32]!
    var edgeIds: [Int32]!
    var surfaceIds: [Int32]!
    
    var restVol: [Float]!
    var edgeLengths: [Float]!
    var invMass: [Float]!
    
    var edgeCompliance: Float!
    var volCompliance: Float!
    
    var temp: [Float]!
    var grads: [Float]!
    
    var grabId: Int = -1
    var grabInvMass: Float = 0.0
    
    let volIdOrder : [[Int]] = [[1, 3, 2], [0, 2, 3], [0, 3, 1], [0, 1, 2]]
    
    // MARK: Metal vars
    
    var pipelineState: MTLRenderPipelineState!
    
    var posBuffer : MTLBuffer!
    var norBuffer : MTLBuffer!
    var indexBuffer : MTLBuffer!
    
    var boundingBox: BoundingBox = BoundingBox(min: .zero, max: .zero)
    
    init(name : String, edgeCompliance edge: Float = 500.0, volCompliance vol: Float = 0.0) {
        super.init()
        
        edgeCompliance = edge
        volCompliance = vol
        
        guard let url = Bundle.main.url(forResource: "bunny", withExtension: "json") else {
            fatalError("Missing bunny.json in bundle")
        }

        do {
            let data = try Data(contentsOf: url, options: .mappedIfSafe)
            let json = try JSONSerialization.jsonObject(with: data, options: [])

            guard let dict = json as? [String: Any] else {
                fatalError("json root must be an object")
            }

            // Convert numbers properly (JSON gives you Double/NSNumber, not Float32/Int32)
            guard let vertsRaw = dict["verts"] as? [NSNumber] else {
                fatalError("json missing/invalid 'verts' (expected array of numbers)")
            }
            guard let tetIdsRaw = dict["tetIds"] as? [NSNumber] else {
                fatalError("json missing/invalid 'tetIds' (expected array of numbers)")
            }
            guard let tetEdgeIdsRaw = dict["tetEdgeIds"] as? [NSNumber] else {
                fatalError("json missing/invalid 'tetEdgeIds' (expected array of numbers)")
            }
            guard let tetSurfaceTriIdsRaw = dict["tetSurfaceTriIds"] as? [NSNumber] else {
                fatalError("json missing/invalid 'tetSurfaceTriIds' (expected array of numbers)")
            }

            let verts: [Float32] = vertsRaw.map { $0.floatValue }
            let tetIds: [Int32] = tetIdsRaw.map { $0.int32Value }
            let tetEdgeIds: [Int32] = tetEdgeIdsRaw.map { $0.int32Value }
            let tetSurfaceTriIds: [Int32] = tetSurfaceTriIdsRaw.map { $0.int32Value }

            // Optional shape sanity checks (crash early if malformed)
            guard verts.count % 3 == 0 else { fatalError("'verts' count must be multiple of 3") }
            guard tetIds.count % 4 == 0 else { fatalError("'tetIds' count must be multiple of 4") }
            guard tetEdgeIds.count % 2 == 0 else { fatalError("'tetEdgeIds' count must be multiple of 2") }
            guard tetSurfaceTriIds.count % 3 == 0 else { fatalError("'tetSurfaceTriIds' count must be multiple of 2") }

            print("Successfully loaded JSON")

            numParticles = verts.count / 3
            numTets = tetIds.count / 4

            pos = verts
            prevPos = pos
            vel = Array<Float>(repeating: 0, count: 3 * numParticles)
            
            normals = Array<Float>()

            self.tetIds = tetIds
            self.edgeIds = tetEdgeIds
            self.surfaceIds = tetSurfaceTriIds // keep/use if you need it (store it if you have a property)

            self.restVol = Array<Float>(repeating: 0, count: numTets)
            self.edgeLengths = Array<Float>(repeating: 0, count: edgeIds.count / 2)
            self.invMass = Array<Float>(repeating: 0, count: numParticles)

            temp = Array<Float>(repeating: 0, count: 4 * 3)
            grads = Array<Float>(repeating: 0, count: 4 * 3)
        } catch {
            fatalError("Could not load bunny.json: \(error)")
        }
        
        initPhysics()
        rebuildBoundingBox()
        
        // initialize softbody as a metal mesh
        pipelineState = SoftBody.buildPipelineState()
        
        posBuffer = Renderer.device.makeBuffer(
            bytes: pos,
            length: pos.count * MemoryLayout<Float>.stride,
            options: .storageModeShared
        )
        
        indexBuffer = Renderer.device.makeBuffer(
            bytes: surfaceIds,
            length: surfaceIds.count * MemoryLayout<Int32>.stride,
            options: .storageModeShared
        )
        
        SoftBody.calculateNormals(surfaceIds: surfaceIds, pos: pos, normals: &normals)
        
        norBuffer = Renderer.device.makeBuffer(
            bytes: normals,
            length: normals.count * MemoryLayout<Float>.stride,
            options: .storageModeShared
        )
    }
    
    private static func buildPipelineState() -> MTLRenderPipelineState {
        let library = Renderer.library
        let vertexFunction = library?.makeFunction(name: "vertex_main_softbody")
        let fragmentFunction = library?.makeFunction(name: "fragment_main_softbody")
        
        var pipelineState: MTLRenderPipelineState
        
        let pipelineDescriptor = MTLRenderPipelineDescriptor()
        pipelineDescriptor.vertexFunction = vertexFunction
        pipelineDescriptor.fragmentFunction = fragmentFunction
        
        let vertexDescriptor = MTLVertexDescriptor()
        
        // position
        vertexDescriptor.attributes[0].format = .float3
        vertexDescriptor.attributes[0].offset = 0
        vertexDescriptor.attributes[0].bufferIndex = 0
        vertexDescriptor.layouts[0].stride = 3 * MemoryLayout<Float>.stride
        
        pipelineDescriptor.vertexDescriptor = vertexDescriptor
        
        pipelineDescriptor.colorAttachments[0].pixelFormat = .bgra8Unorm
        pipelineDescriptor.depthAttachmentPixelFormat = .depth32Float
        
        do {
          pipelineState = try Renderer.device.makeRenderPipelineState(descriptor: pipelineDescriptor)
        } catch let error {
          fatalError(error.localizedDescription)
        }
        
        return pipelineState
    }
    
    private static func calculateNormals(
        surfaceIds: [Int32],
        pos: [Float],
        normals: inout [Float]
    ) {
        normals = Array<Float>()
        normals.reserveCapacity(surfaceIds.count)

        for i in stride(from: 0, to: surfaceIds.count, by: 3) {

            let i0 = Int(surfaceIds[i])     * 3
            let i1 = Int(surfaceIds[i + 1]) * 3
            let i2 = Int(surfaceIds[i + 2]) * 3

            let v0 = SIMD3<Float>(
                pos[i0],
                pos[i0 + 1],
                pos[i0 + 2]
            )

            let v1 = SIMD3<Float>(
                pos[i1],
                pos[i1 + 1],
                pos[i1 + 2]
            )

            let v2 = SIMD3<Float>(
                pos[i2],
                pos[i2 + 1],
                pos[i2 + 2]
            )

            let n = normalize(cross(v1 - v0, v2 - v0))

            normals.append(n.x)
            normals.append(n.y)
            normals.append(n.z)
        }
    }
    
    func rebuildBoundingBox() {
        boundingBox.min = SIMD3<Float>(.infinity, .infinity, .infinity)
        boundingBox.max = SIMD3<Float>(-.infinity, -.infinity, -.infinity)
        
        for i in 0..<surfaceIds.count {
            let posArrayIndex = Int(surfaceIds[i]) * 3
            let vertex = SIMD3<Float>(
                x: pos[posArrayIndex],
                y: pos[posArrayIndex + 1],
                z: pos[posArrayIndex + 2]
            )
            
            boundingBox.min = min(boundingBox.min, vertex)
            boundingBox.max = max(boundingBox.max, vertex)
        }
    }
    
    func simulate(dt: Double) {
        let gravity = SIMD3<Float>(0, -9.81, 0)
        let sdt : Float = Float(dt) / 10.0
        
        for _ in 0..<10 {
            preSolve(dt: sdt, gravity: [gravity.x, gravity.y, gravity.z])
            solve(dt: sdt)
            postSolve(dt: sdt)
        }
    }
    
    func initPhysics() {
        restVol = Array<Float>(repeating: 0, count: numTets)
        invMass = Array<Float>(repeating: 0, count: numParticles)
        
        for i in 0..<numTets {
            let vol = getTetVolume(nr: i)
            restVol[i] = vol
            let pInvMass = vol > 0.0 ? 1.0 / (vol / 4.0) : 0.0
            invMass[Int(tetIds[4 * i])] += pInvMass
            invMass[Int(tetIds[4 * i + 1])] += pInvMass
            invMass[Int(tetIds[4 * i + 2])] += pInvMass
            invMass[Int(tetIds[4 * i + 3])] += pInvMass
        }
        for i in 0..<edgeLengths.count {
            let id0 = Int(edgeIds[2 * i])
            let id1 = Int(edgeIds[2 * i + 1])
            edgeLengths[i] = sqrt(vecDistSquared(pos, id0, pos, id1))
        }
    }
    
    func updateMeshes() {
        SoftBody.calculateNormals(surfaceIds: surfaceIds, pos: pos, normals: &normals)
        posBuffer.contents().copyMemory(from: pos, byteCount: pos.count * MemoryLayout<Float>.stride)
        norBuffer.contents().copyMemory(from: normals, byteCount: normals.count * MemoryLayout<Float>.stride)
        rebuildBoundingBox()
    }
    
    func translate(x : Float, y : Float, z : Float) {
        for i in 0..<self.numParticles {
            vecAdd(&pos, i, [x, y, z], 0)
            vecAdd(&prevPos, i, [x, y, z], 0)
        }
    }
    
    func getTetVolume(nr: Int) -> Float {
        let id0 = Int(tetIds[4 * nr])
        let id1 = Int(tetIds[4 * nr + 1])
        let id2 = Int(tetIds[4 * nr + 2])
        let id3 = Int(tetIds[4 * nr + 3])
        
        vecSetDiff(&temp, 0, pos, id1, pos, id0)
        vecSetDiff(&temp, 1, pos, id2, pos, id0)
        vecSetDiff(&temp, 2, pos, id3, pos, id0)
        vecSetCross(&temp, 3, temp, 0, temp, 1)
        
        return vecDot(temp, 3, temp, 2) / 6.0
    }
    
    func preSolve(dt: Float, gravity: [Float]) {
        for i in 0..<self.numParticles {
            if (invMass[i] == 0.0) {
                continue
            }
            vecAdd(&vel, i, gravity, 0, dt)
            vecCopy(&prevPos, i, pos, i)
            vecAdd(&pos, i, vel, i, dt)
            let y = pos[3 * i + 1]
            if (y < 0.0) {
                vecCopy(&pos, i, prevPos, i)
                pos[3 * i + 1] = 0.0
            }
        }
    }
    
    func solve(dt: Float) {
        solveEdges(compliance: self.edgeCompliance, dt: dt)
        solveVolumes(compliance: self.volCompliance, dt: dt)
    }
    
    func postSolve(dt: Float) {
        for i in 0..<self.numParticles {
            if (invMass[i] == 0.0) {
                continue
            }
            vecSetDiff(&vel, i, pos, i, prevPos, i, 1.0 / dt)
        }
    }
    
    func solveEdges(compliance: Float, dt: Float) {
        let alpha = compliance / dt / dt
        
        for i in 0..<edgeLengths.count {
            let id0 = Int(edgeIds[2 * i])
            let id1 = Int(edgeIds[2 * i + 1])
            let w0 = invMass[id0]
            let w1 = invMass[id1]
            let w = w0 + w1
            if (w == 0.0) {
                continue
            }
            
            vecSetDiff(&grads, 0, pos, id0, pos, id1)
            let len = sqrt(vecLengthSquared(grads, 0))
            if (len == 0.0) {
                continue
            }
            vecScale(&grads, 0, 1.0 / len)
            let restLen: Float = edgeLengths[i]
            let C: Float = len - restLen
            let s: Float = -C / (w + alpha)
            vecAdd(&pos, id0, grads, 0, s * w0)
            vecAdd(&pos, id1, grads, 0, -s * w1)
        }
    }
    
    func solveVolumes(compliance: Float, dt: Float) {
        let alpha = compliance / dt / dt
        
        for i in 0..<self.numTets {
            var w: Float = 0.0
            
            for j in 0..<4 {
                let id0 = Int(self.tetIds[4 * i + self.volIdOrder[j][0]])
                let id1 = Int(self.tetIds[4 * i + self.volIdOrder[j][1]])
                let id2 = Int(self.tetIds[4 * i + self.volIdOrder[j][2]])
                
                vecSetDiff(&temp, 0, pos, id1, pos, id0)
                vecSetDiff(&temp, 1, pos, id2, pos, id0)
                vecSetCross(&grads, j, temp, 0, temp, 1)
                vecScale(&grads, j, 1.0 / 6.0)
                
                w += invMass[Int(tetIds[4 * i + j])] * vecLengthSquared(grads, j)
            }
            if w == 0.0 {
                continue
            }
            
            let vol: Float = getTetVolume(nr: i)
            let restVol: Float = self.restVol[i]
            let C: Float = vol - restVol
            let s: Float = -C / (w + alpha)
            
            for j in 0..<4 {
                let id: Int = Int(tetIds[4 * i + j])
                vecAdd(&pos, id, grads, j, s * invMass[id])
            }
        }
    }
    
    func startGrab(idx: Int, pos: SIMD3<Float>) {
        grabId = idx
        let p = [pos.x, pos.y, pos.z]
        
        if grabId >= 0 {
            grabInvMass = invMass[grabId]
            invMass[grabId] = 0.0
            vecCopy(&self.pos, grabId, p, 0)
        }
    }
    
    func updateGrab(pos: SIMD3<Float>, vel: SIMD3<Float>) {
        if self.grabId >= 0 {
            let p = [pos.x, pos.y, pos.z]
            vecCopy(&self.pos, self.grabId, p, 0)
        }
    }
    
    func endGrab(pos: SIMD3<Float>, vel: SIMD3<Float>) {
//        if self.grabId >= 0 {
//            invMass[grabId] = grabInvMass
//            let v = [vel.x, vel.y, vel.z]
//            vecCopy(&self.vel, grabId, v, 0)
//        }
//        grabId = -1
    }
}
