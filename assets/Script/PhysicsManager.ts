/*
 * @Author: conjurer
 * @Github: https://github.com/dot123
 * @Date: 2021-02-05 23:02:27
 * @LastEditors: conjurer
 * @LastEditTime: 2021-02-22 20:20:17
 * @Description:
 */

function remove(array, value) {
    let index = array.indexOf(value);
    if (index >= 0) {
        array.splice(index, 1);
        return true;
    }
    return false;
}

export interface Host {
    onBeginContact(contact: PhysicsContact, selfCollider: PhysicsCollider, otherCollider: PhysicsCollider);
    onEndContact(contact: PhysicsContact, selfCollider: PhysicsCollider, otherCollider: PhysicsCollider);
    onPreSolve(contact: PhysicsContact, selfCollider: PhysicsCollider, otherCollider: PhysicsCollider);
    onPostSolve(contact: PhysicsContact, selfCollider: PhysicsCollider, otherCollider: PhysicsCollider);
}

const box2d = require("box2d");
let b2_aabb_tmp = new box2d.b2AABB();
let b2_vec2_tmp1 = new box2d.b2Vec2();
let b2_vec2_tmp2 = new box2d.b2Vec2();

enum ContactType {
    BEGIN_CONTACT = "begin-contact",
    END_CONTACT = "end-contact",
    PRE_SOLVE = "pre-solve",
    POST_SOLVE = "post-solve",
}

export enum RigidBodyType {
    /**
     * 零质量，零速度，可以手动移动。
     * @property {Number} Static
     */
    Static = 0,
    /**
     * 零质量，可以被设置速度。
     * @property {Number} Kinematic
     */
    Kinematic = 1,
    /**
     * 有质量，可以设置速度，力等。
     * @property {Number} Dynamic
     */
    Dynamic = 2,
    /**
     * Kinematic 类型的扩展，可以被动画控制动画效果。
     * @property {Number} Animated
     */
    Animated = 3,
}

export enum RayCastType {
    /**
     * 检测射线路径上最近的碰撞体
     * @property {Number} Closest
     */
    Closest = 0,
    /**
     * 检测射线路径上任意的碰撞体。
     * 一旦检测到任何碰撞体，将立刻结束检测其他的碰撞体。
     * @property {Number} Any
     */
    Any = 1,
    /**
     * 检测射线路径上所有的碰撞体。
     * 同一个碰撞体上有可能会返回多个碰撞点(因为一个碰撞体可能由多个夹具组成，每一个夹具会返回一个碰撞点，碰撞点有可能在碰撞体内部)，AllClosest 删选同一个碰撞体上最近的哪一个碰撞点。
     * @property {Number} AllClosest
     */
    AllClosest = 2,

    /**
     * 检测射线路径上所有的碰撞体。
     * 同一个碰撞体上有可能会返回多个碰撞点，All 将返回所有这些碰撞点。
     * @property {Number} All
     */
    All = 3,
}

/**
 * 物理单位与像素单位互相转换的比率，一般是 32。
 * @property {Number} PTM_RATIO
 * @static
 */
const PTM_RATIO = 32;

const ANGLE_TO_PHYSICS_ANGLE = -0.017453292519943295;
const PHYSICS_ANGLE_TO_ANGLE = -57.29577951308232;

export class PhysicsManager {
    /**
     * 速度更新迭代数
     * @property {Number} VELOCITY_ITERATIONS
     * @default 10
     * @static
     */
    static VELOCITY_ITERATIONS = 10;

    /**
     * 位置迭代更新数
     * @property {Number} POSITION_ITERATIONS
     * @default 10
     * @static
     */
    static POSITION_ITERATIONS = 10;

    /**
     * 指定固定的物理更新间隔时间，需要开启 enabledAccumulator 才有效。
     * @property {Number} FIXED_TIME_STEP
     * @default 1/60
     * @static
     */
    static FIXED_TIME_STEP = 1 / 60;

    /**
     * 每次可用于更新物理系统的最大时间，需要开启 enabledAccumulator 才有效。
     * @property {Number} MAX_ACCUMULATOR
     * @default 1/5
     * @static
     */
    static MAX_ACCUMULATOR = 1 / 5;

    static DrawBits = box2d.b2DrawFlags;

    static CollisionMatrix = [[true]];

    static TIMESTEP = 60;

    private _debugDrawFlags = 0;

    private _world = null;

    private _bodies = [];
    private _joints = [];

    private _delayEvents = [];

    private _accumulator = 0;

    private _steping = false;

    /**
     * 如果开启此选项，那么将会以固定的间隔时间 FIXED_TIME_STEP 来更新物理引擎，如果一个 update 的间隔时间大于 FIXED_TIME_STEP，则会对物理引擎进行多次更新。
     * 如果关闭此选项，那么将会根据设定的 frame rate 计算出一个间隔时间来更新物理引擎。
     * @property {Boolean} enabledAccumulator
     * @default false
     */
    public enabledAccumulator = false;

    private _contactListener: PhysicsContactListener = null;
    private _aabbQueryCallback: PhysicsAABBQueryCallback = null;
    private _raycastQueryCallback: PhysicsRayCastCallback = null;

    _pushDelayEvent(target, func, args) {
        if (this._steping) {
            this._delayEvents.push({
                target: target,
                func: func,
                args: args,
            });
        } else {
            target[func].apply(target, args);
        }
    }

    update(dt) {
        if (!this._world || !this.enabled) return;

        this._steping = true;

        if (this.enabledAccumulator) {
            this._accumulator += dt;

            // max accumulator time to avoid spiral of death
            if (this._accumulator > PhysicsManager.MAX_ACCUMULATOR) {
                this._accumulator = PhysicsManager.MAX_ACCUMULATOR;
            }

            while (this._accumulator > PhysicsManager.FIXED_TIME_STEP) {
                this._world.Step(PhysicsManager.FIXED_TIME_STEP, PhysicsManager.VELOCITY_ITERATIONS, PhysicsManager.POSITION_ITERATIONS);
                this._accumulator -= PhysicsManager.FIXED_TIME_STEP;
            }
        } else {
            let timeStep = dt;
            this._world.Step(timeStep, PhysicsManager.VELOCITY_ITERATIONS, PhysicsManager.POSITION_ITERATIONS);
        }

        if (this._debugDrawFlags) {
            this._world.DrawDebugData();
        }

        this._steping = false;

        let events = this._delayEvents;
        for (let i = 0, l = events.length; i < l; i++) {
            let event = events[i];
            event.target[event.func].apply(event.target, event.args);
        }
        events.length = 0;
    }

    /**
     * 获取包含给定世界坐标系点的碰撞体
     * @method testPoint
     * @param {Vec2} point - the world point
     * @return {PhysicsCollider}
     */
    testPoint(x, y) {
        x = b2_vec2_tmp1.x = x / PTM_RATIO;
        y = b2_vec2_tmp1.y = y / PTM_RATIO;

        let d = 0.2 / PTM_RATIO;
        b2_aabb_tmp.lowerBound.x = x - d;
        b2_aabb_tmp.lowerBound.y = y - d;
        b2_aabb_tmp.upperBound.x = x + d;
        b2_aabb_tmp.upperBound.y = y + d;

        let callback = this._aabbQueryCallback;
        callback.init(b2_vec2_tmp1);
        this._world.QueryAABB(callback, b2_aabb_tmp);

        let fixture = callback.getFixture();
        if (fixture) {
            return fixture.collider;
        }

        return null;
    }

    /**
     * 获取与给定世界坐标系矩形相交的碰撞体
     * @method testAABB
     * @param {Rect} rect - the world rect
     * @return {[PhysicsCollider]}
     */
    testAABB(rect) {
        b2_aabb_tmp.lowerBound.x = rect.xMin / PTM_RATIO;
        b2_aabb_tmp.lowerBound.y = rect.yMin / PTM_RATIO;
        b2_aabb_tmp.upperBound.x = rect.xMax / PTM_RATIO;
        b2_aabb_tmp.upperBound.y = rect.yMax / PTM_RATIO;

        let callback = this._aabbQueryCallback;
        callback.init();
        this._world.QueryAABB(callback, b2_aabb_tmp);

        let fixtures = callback.getFixtures();

        return fixtures;
    }

    /**
     * 检测哪些碰撞体在给定射线的路径上，射线检测将忽略包含起始点的碰撞体。
     * @method rayCast
     * @param {Vec2} p1 - start point of the raycast
     * @param {Vec2} p2 - end point of the raycast
     * @param {RayCastType} type - optional, default is RayCastType.Closest
     * @return {[PhysicsRayCastResult]}
     */
    rayCast(x1, y1, x2, y2, type) {
        if (x1 == x2 && y1 == y2) {
            return [];
        }

        type = type || RayCastType.Closest;

        b2_vec2_tmp1.x = x1 / PTM_RATIO;
        b2_vec2_tmp1.y = y1 / PTM_RATIO;
        b2_vec2_tmp2.x = x2 / PTM_RATIO;
        b2_vec2_tmp2.y = y2 / PTM_RATIO;

        let callback = this._raycastQueryCallback;
        callback.init(type);
        this._world.RayCast(callback, b2_vec2_tmp1, b2_vec2_tmp2);

        let fixtures = callback.getFixtures();
        if (fixtures.length > 0) {
            let points = callback.getPoints();
            let normals = callback.getNormals();
            let fractions = callback.getFractions();

            let results = [];
            for (let i = 0, l = fixtures.length; i < l; i++) {
                let fixture = fixtures[i];
                let collider = fixture.collider;

                if (type === RayCastType.AllClosest) {
                    let result = results.find(function (result) {
                        return result.collider === collider;
                    });

                    if (result) {
                        if (fractions[i] < result.fraction) {
                            result.fixtureIndex = collider._getFixtureIndex(fixture);
                            result.point.x = points[i].x * PTM_RATIO;
                            result.point.y = points[i].y * PTM_RATIO;
                            result.normal.x = normals[i].x;
                            result.normal.y = normals[i].y;
                            result.fraction = fractions[i];
                        }
                        continue;
                    }
                }

                results.push({
                    collider: collider,
                    fixtureIndex: collider._getFixtureIndex(fixture),
                    point: { x: points[i].x * PTM_RATIO, y: points[i].y * PTM_RATIO },
                    normal: { x: normals[i].x, y: normals[i].y },
                    fraction: fractions[i],
                });
            }

            return results;
        }

        return [];
    }

    _registerContactFixture(fixture) {
        this._contactListener.registerContactFixture(fixture);
    }

    _unregisterContactFixture(fixture) {
        this._contactListener.unregisterContactFixture(fixture);
    }

    _addBody(body, bodyDef) {
        let world = this._world;
        if (!world) return;

        body._b2Body = world.CreateBody(bodyDef);
        body._b2Body.body = body;

        this._bodies.push(body);
    }

    _removeBody(body) {
        let world = this._world;
        if (!world) return;

        body._b2Body.body = null;
        world.DestroyBody(body._b2Body);
        body._b2Body = null;

        remove(this._bodies, body);
    }

    _addJoint(joint, jointDef) {
        let b2joint = this._world.CreateJoint(jointDef);
        if (!b2joint) return;

        b2joint._joint = joint;
        joint._joint = b2joint;

        this._joints.push(joint);
    }

    _removeJoint(joint) {
        if (joint._isValid()) {
            this._world.DestroyJoint(joint._joint);
        }

        if (joint._joint) {
            joint._joint._joint = null;
        }

        remove(this._joints, joint);
    }

    _initCallback() {
        if (!this._world) {
            console.warn("Please init PhysicsManager first");
            return;
        }

        if (this._contactListener) return;

        let listener = new PhysicsContactListener();
        listener.setBeginContact(this._onBeginContact);
        listener.setEndContact(this._onEndContact);
        listener.setPreSolve(this._onPreSolve);
        listener.setPostSolve(this._onPostSolve);
        this._world.SetContactListener(listener);

        this._contactListener = listener;

        this._aabbQueryCallback = new PhysicsAABBQueryCallback();
        this._raycastQueryCallback = new PhysicsRayCastCallback();
    }

    _getWorld() {
        return this._world;
    }

    _onBeginContact(b2contact) {
        let c = PhysicsContact._get(b2contact);
        c._emit(ContactType.BEGIN_CONTACT);
    }

    _onEndContact(b2contact) {
        let c = b2contact._contact;
        if (!c) {
            return;
        }
        c._emit(ContactType.END_CONTACT);

        PhysicsContact._put(b2contact);
    }

    _onPreSolve(b2contact) {
        let c = b2contact._contact;
        if (!c) {
            return;
        }

        c._emit(ContactType.PRE_SOLVE);
    }

    _onPostSolve(b2contact, impulse) {
        let c = b2contact._contact;
        if (!c) {
            return;
        }

        // impulse only survive during post sole callback
        c._impulse = impulse;
        c._emit(ContactType.POST_SOLVE);
        c._impulse = null;
    }

    /**
     * 设置调试绘制标志
     * @property {Number} debugDrawFlags
     * @default 0
     * @example
     * // enable all debug draw info
     * let Bits = PhysicsManager.DrawBits;
     * PhysicsManager.debugDrawFlags = Bits.e_aabbBit |
        Bits.e_pairBit |
        Bits.e_centerOfMassBit |
        Bits.e_jointBit |
        Bits.e_shapeBit;
    
    * // disable debug draw info
    * PhysicsManager.debugDrawFlags = 0;
    */
    debugDrawFlags(value, debugDrawer) {
        if (value) {
            let debugDraw = new PhysicsDebugDraw(debugDrawer);
            debugDraw.SetFlags(this.debugDrawFlags);
            this._world.SetDebugDraw(debugDraw);
            this._world.m_debugDraw.SetFlags(value);
        } else {
            this._world.SetDebugDraw(null);
        }

        this._debugDrawFlags = value;
    }

    /**
     * 指定是否启用物理系统？
     * @property {Boolean} enabled
     */
    private _enabled = false;

    get enabled() {
        return this._enabled;
    }

    set enabled(value) {
        if (value && !this._world) {
            let world = new box2d.b2World(new box2d.b2Vec2(0, -10));
            world.SetAllowSleeping(true);

            this._world = world;

            this._initCallback();
        }

        this._enabled = value;
    }

    get randSeed() {
        if (this._world) {
            return this._world.GetRandSeed();
        }
        return -1;
    }

    set randSeed(value) {
        if (this._world) {
            this._world.SetRandSeed(value);
        }
    }

    /**
     * 物理世界重力值
     * @property {Vec2} gravity
     */
    get gravity() {
        if (this._world) {
            let g = this._world.GetGravity();
            return { x: g.x * PTM_RATIO, y: g.y * PTM_RATIO };
        }
        return { x: 0, y: 0 };
    }

    set gravity(value) {
        if (this._world) {
            this._world.SetGravity(new box2d.b2Vec2(value.x / PTM_RATIO, value.y / PTM_RATIO));
        }
    }

    private static _physicsManager = null;

    static getPhysicsManager() {
        if (!PhysicsManager._physicsManager) {
            PhysicsManager._physicsManager = new PhysicsManager();
        }
        return PhysicsManager._physicsManager;
    }

    /**
     * 清理物理世界
     */
    clear() {
        if (this._world) {
            this._bodies.length = 0;
            this._joints.length = 0;
            this._delayEvents.length = 0;

            this.enabledAccumulator = false;
            this._steping = false;
            this._accumulator = 0;

            this._debugDrawFlags = 0;
            this._world.SetDebugDraw(null);

            this._world.SetContactListener(null);
            this._contactListener = null;

            this._aabbQueryCallback = null;
            this._raycastQueryCallback = null;

            this._world = null;
            this._enabled = false;
        }
    }
}

let pools = [];

// temp world manifold
let pointCache = [
    { x: 0, y: 0 },
    { x: 0, y: 0 },
];

let b2worldmanifold = new box2d.b2WorldManifold();

let worldmanifold = {
    /**
     * 碰撞点集合
     * @property {[Vec2]} points
     */
    points: [],

    /**
     * 一个负数，用于指明重叠的部分
     */
    separations: [],

    /**
     * 世界坐标系下由 A 指向 B 的向量
     * @property {Vec2} normal
     */
    normal: { x: 0, y: 0 },
};

/**
 * ManifoldPoint 是接触信息中的接触点信息。它拥有关于几何和接触点的详细信息。
 * 注意：信息中的冲量用于系统内部缓存，提供的接触力可能不是很准确，特别是高速移动中的碰撞信息。
 * @class ManifoldPoint
 */
/**
 * 本地坐标点的用途取决于 manifold 的类型
 * - e_circles: circleB 的本地中心点
 * - e_faceA: circleB 的本地中心点 或者是 polygonB 的截取点
 * - e_faceB: polygonB 的截取点
 * @property {Vec2} localPoint
 */
/**
 * 法线冲量。
 * @property {Number} normalImpulse
 */
/**
 * 切线冲量。
 * @property {Number} tangentImpulse
 */
function ManifoldPoint() {
    this.localPoint = { x: 0, y: 0 };
    this.normalImpulse = 0;
    this.tangentImpulse = 0;
}

let manifoldPointCache = [new ManifoldPoint(), new ManifoldPoint()];

let manifold = {
    /**
     * Manifold 类型 :  0: e_circles, 1: e_faceA, 2: e_faceB
     * @property {Number} type
     */
    type: 0,

    /**
     * !#en
     * The local point usage depends on the manifold type:
     * -e_circles: the local center of circleA
     * -e_faceA: the center of faceA
     * -e_faceB: the center of faceB
     * !#zh
     * 用途取决于 manifold 类型
     * -e_circles: circleA 的本地中心点
     * -e_faceA: faceA 的本地中心点
     * -e_faceB: faceB 的本地中心点
     * @property {Vec2} localPoint
     */
    localPoint: { x: 0, y: 0 },
    /**
     * !#en
     * -e_circles: not used
     * -e_faceA: the normal on polygonA
     * -e_faceB: the normal on polygonB
     * !#zh
     * -e_circles: 没被使用到
     * -e_faceA: polygonA 的法向量
     * -e_faceB: polygonB 的法向量
     * @property {Vec2} localNormal
     */
    localNormal: { x: 0, y: 0 },

    /**
     * !#en
     * the points of contact.
     * !#zh
     * 接触点信息。
     * @property {[ManifoldPoint]} points
     */
    points: [],
};

/**
 * 用于返回给回调的接触冲量。
 * @class PhysicsImpulse
 */
let impulse = {
    /**
     * 法线方向的冲量
     * @property normalImpulses
     */
    normalImpulses: [],
    /**
     * 切线方向的冲量
     * @property tangentImpulses
     */
    tangentImpulses: [],
};

/**
 * 物理接触会在开始和结束碰撞之间生成，并作为参数传入到碰撞回调函数中。
 * 注意：传入的物理接触会被系统进行重用，所以不要在使用中缓存里面的任何信息。
 * @class PhysicsContact
 */
export class PhysicsContact {
    public colliderA = null;
    public colliderB = null;
    public disabled = false;
    public disabledOnce = false;

    private _impulse = null;
    private _inverted = false;
    private _b2contact = null;

    _init(b2contact) {
        this.colliderA = b2contact.GetFixtureA().collider;
        this.colliderB = b2contact.GetFixtureB().collider;
        this.disabled = false;
        this.disabledOnce = false;
        this._impulse = null;

        this._inverted = false;

        this._b2contact = b2contact;
        b2contact._contact = this;
    }

    _reset() {
        this.setTangentSpeed(0);
        this.resetFriction();
        this.resetRestitution();

        this.colliderA = null;
        this.colliderB = null;
        this.disabled = false;
        this._impulse = null;

        this._b2contact._contact = null;
        this._b2contact = null;
    }

    /**
     * 获取世界坐标系下的碰撞信息。
     * @method getWorldManifold
     * @return {WorldManifold}
     */
    getWorldManifold() {
        let points = worldmanifold.points;
        let separations = worldmanifold.separations;
        let normal = worldmanifold.normal;

        this._b2contact.GetWorldManifold(b2worldmanifold);
        let b2points = b2worldmanifold.points;
        let b2separations = b2worldmanifold.separations;

        let count = this._b2contact.GetManifold().pointCount;
        points.length = separations.length = count;

        for (let i = 0; i < count; i++) {
            let p = pointCache[i];
            p.x = b2points[i].x * PTM_RATIO;
            p.y = b2points[i].y * PTM_RATIO;

            points[i] = p;
            separations[i] = b2separations[i] * PTM_RATIO;
        }

        normal.x = b2worldmanifold.normal.x;
        normal.y = b2worldmanifold.normal.y;

        if (this._inverted) {
            normal.x *= -1;
            normal.y *= -1;
        }

        return worldmanifold;
    }

    /**
     * 获取本地（局部）坐标系下的碰撞信息。
     * @method getManifold
     * @return {Manifold}
     */
    getManifold() {
        let points = manifold.points;
        let localNormal = manifold.localNormal;
        let localPoint = manifold.localPoint;

        let b2manifold = this._b2contact.GetManifold();
        let b2points = b2manifold.points;
        let count = (points.length = b2manifold.pointCount);

        for (let i = 0; i < count; i++) {
            let p = manifoldPointCache[i];
            let b2p = b2points[i];
            p.localPoint.x = b2p.localPoint.x * PTM_RATIO;
            p.localPoint.Y = b2p.localPoint.Y * PTM_RATIO;
            p.normalImpulse = b2p.normalImpulse * PTM_RATIO;
            p.tangentImpulse = b2p.tangentImpulse;

            points[i] = p;
        }

        localPoint.x = b2manifold.localPoint.x * PTM_RATIO;
        localPoint.y = b2manifold.localPoint.y * PTM_RATIO;
        localNormal.x = b2manifold.localNormal.x;
        localNormal.y = b2manifold.localNormal.y;
        manifold.type = b2manifold.type;

        if (this._inverted) {
            localNormal.x *= -1;
            localNormal.y *= -1;
        }

        return manifold;
    }

    /**
     * 获取冲量信息
     * 注意：这个信息只有在 onPostSolve 回调中才能获取到
     * @method getImpulse
     * @return {PhysicsImpulse}
     */
    getImpulse() {
        let b2impulse = this._impulse;
        if (!b2impulse) return null;

        let normalImpulses = impulse.normalImpulses;
        let tangentImpulses = impulse.tangentImpulses;
        let count = b2impulse.count;
        for (let i = 0; i < count; i++) {
            normalImpulses[i] = b2impulse.normalImpulses[i] * PTM_RATIO;
            tangentImpulses[i] = b2impulse.tangentImpulses[i];
        }

        tangentImpulses.length = normalImpulses.length = count;

        return impulse;
    }

    _emit(contactType) {
        let func;
        switch (contactType) {
            case ContactType.BEGIN_CONTACT:
                func = "onBeginContact";
                break;
            case ContactType.END_CONTACT:
                func = "onEndContact";
                break;
            case ContactType.PRE_SOLVE:
                func = "onPreSolve";
                break;
            case ContactType.POST_SOLVE:
                func = "onPostSolve";
                break;
        }

        let colliderA = this.colliderA;
        let colliderB = this.colliderB;

        let bodyA = colliderA.body;
        let bodyB = colliderB.body;

        let host = bodyA.host;
        if (bodyA.enabledContactListener && host) {
            this._inverted = false;
            if (bodyA._inited && host[func]) {
                host[func](this, colliderA, colliderB);
            }
        }

        host = bodyB.host;
        if (bodyB.enabledContactListener && host) {
            this._inverted = true;
            if (bodyB._inited && host[func]) {
                host[func](this, colliderB, colliderA);
            }
        }

        if (this.disabled || this.disabledOnce) {
            this.setEnabled(false);
            this.disabledOnce = false;
        }
    }

    static _get(b2contact) {
        let c;
        if (pools.length === 0) {
            c = new PhysicsContact();
        } else {
            c = pools.pop();
        }

        c._init(b2contact);
        return c;
    }

    static _put(b2contact) {
        let c = b2contact._contact;
        if (!c) return;

        pools.push(c);
        c._reset();
    }

    /**
     * 如果 disabled 被设置为 true，那么直到接触结束此接触都将被忽略。
     * 如果只是希望在当前时间步或子步中忽略此接触，请使用 disabledOnce 。
     * @property {Boolean} disabled
     */
    /**
     * 在当前时间步或子步中忽略此接触。
     * @property {Boolean} disabledOnce
     */
    setEnabled(value) {
        this._b2contact.SetEnabled(value);
    }

    /**
     * 返回碰撞体是否已经接触到。
     * @method isTouching
     * @return {Boolean}
     */
    isTouching() {
        return this._b2contact.IsTouching();
    }

    /**
     * 为传送带设置期望的切线速度
     * @method setTangentSpeed
     * @param {Number} tangentSpeed
     */
    setTangentSpeed(value) {
        this._b2contact.SetTangentSpeed(value / PTM_RATIO);
    }

    /**
     * 获取切线速度
     * @method getTangentSpeed
     * @return {Number}
     */
    getTangentSpeed() {
        return this._b2contact.GetTangentSpeed() * PTM_RATIO;
    }

    /**
     * 覆盖默认的摩擦力系数。你可以在 onPreSolve 回调中调用此函数。
     * @method setFriction
     * @param {Number} friction
     */
    setFriction(value) {
        this._b2contact.SetFriction(value);
    }

    /**
     * 获取当前摩擦力系数
     * @method getFriction
     * @return {Number}
     */
    getFriction() {
        return this._b2contact.GetFriction();
    }

    /**
     * 重置摩擦力系数到默认值
     * @method resetFriction
     */
    resetFriction() {
        return this._b2contact.ResetFriction();
    }

    /**
     * 覆盖默认的恢复系数。你可以在 onPreSolve 回调中调用此函数。
     * @method setRestitution
     * @param {Number} restitution
     */
    setRestitution(value) {
        this._b2contact.SetRestitution(value);
    }

    /**
     * 获取当前恢复系数
     * @method getRestitution
     * @return {Number}
     */
    getRestitution() {
        return this._b2contact.GetRestitution();
    }

    /**
     * 重置恢复系数到默认值
     * @method resetRestitution
     */
    resetRestitution() {
        return this._b2contact.ResetRestitution();
    }
}

class PhysicsAABBQueryCallback {
    private _point = new box2d.b2Vec2();
    private _isPoint = false;
    private _fixtures = [];

    init(point = null) {
        if (point) {
            this._isPoint = true;
            this._point.x = point.x;
            this._point.y = point.y;
        } else {
            this._isPoint = false;
        }

        this._fixtures.length = 0;
    }

    ReportFixture(fixture) {
        if (this._isPoint) {
            if (fixture.TestPoint(this._point)) {
                this._fixtures.push(fixture);
                // We are done, terminate the query.
                return false;
            }
        } else {
            this._fixtures.push(fixture);
        }

        // True to continue the query, false to terminate the query.
        return true;
    }

    getFixture() {
        return this._fixtures[0];
    }
    getFixtures() {
        return this._fixtures;
    }
}

class PhysicsContactListener {
    private _contactFixtures = [];
    private _BeginContact = null;
    private _EndContact = null;
    private _PreSolve = null;
    private _PostSolve = null;

    setBeginContact(cb) {
        this._BeginContact = cb;
    }

    setEndContact(cb) {
        this._EndContact = cb;
    }

    setPreSolve(cb) {
        this._PreSolve = cb;
    }

    setPostSolve(cb) {
        this._PostSolve = cb;
    }

    BeginContact(contact) {
        if (!this._BeginContact) return;

        let fixtureA = contact.GetFixtureA();
        let fixtureB = contact.GetFixtureB();
        let fixtures = this._contactFixtures;

        contact._shouldReport = false;

        if (fixtures.indexOf(fixtureA) !== -1 || fixtures.indexOf(fixtureB) !== -1) {
            contact._shouldReport = true; // for quick check whether this contact should report
            this._BeginContact(contact);
        }
    }

    EndContact(contact) {
        if (this._EndContact && contact._shouldReport) {
            contact._shouldReport = false;
            this._EndContact(contact);
        }
    }

    PreSolve(contact, oldManifold) {
        if (this._PreSolve && contact._shouldReport) {
            this._PreSolve(contact, oldManifold);
        }
    }

    PostSolve(contact, impulse) {
        if (this._PostSolve && contact._shouldReport) {
            this._PostSolve(contact, impulse);
        }
    }

    registerContactFixture(fixture) {
        this._contactFixtures.push(fixture);
    }

    unregisterContactFixture(fixture) {
        remove(this._contactFixtures, fixture);
    }
}

class PhysicsRayCastCallback {
    private _type = 0;
    private _fixtures = [];
    private _points = [];
    private _normals = [];
    private _fractions = [];

    init(type) {
        this._type = type;
        this._fixtures.length = 0;
        this._points.length = 0;
        this._normals.length = 0;
        this._fractions.length = 0;
    }

    ReportFixture(fixture, point, normal, fraction) {
        if (this._type === 0) {
            // closest
            this._fixtures[0] = fixture;
            this._points[0] = point;
            this._normals[0] = normal;
            this._fractions[0] = fraction;
            return fraction;
        }

        this._fixtures.push(fixture);
        this._points.push({ x: point.x, y: point.y });
        this._normals.push({ x: normal.x, y: normal.y });
        this._fractions.push(fraction);

        if (this._type === 1) {
            // any
            return 0;
        } else if (this._type >= 2) {
            // all
            return 1;
        }

        return fraction;
    }

    getFixtures() {
        return this._fixtures;
    }

    getPoints() {
        return this._points;
    }

    getNormals() {
        return this._normals;
    }

    getFractions() {
        return this._fractions;
    }
}

let _tmp_vec2 = { x: 0, y: 0 };

let GREEN_COLOR = { r: 0, g: 255, b: 0, a: 255 };
let RED_COLOR = { r: 255, g: 0, b: 0, a: 255 };

class PhysicsDebugDraw extends box2d.b2Draw {
    private _drawer = null;
    private _xf = null;
    private _dxf = null;

    constructor(drawer) {
        super();
        box2d.b2Draw.call(this);
        this._drawer = drawer;
        this._xf = this._dxf = new box2d.b2Transform();
    }

    _DrawPolygon(vertices, vertexCount) {
        let drawer = this._drawer;

        for (let i = 0; i < vertexCount; i++) {
            box2d.b2Transform.MulXV(this._xf, vertices[i], _tmp_vec2);
            let x = _tmp_vec2.x * PTM_RATIO,
                y = _tmp_vec2.y * PTM_RATIO;
            if (i === 0) drawer.moveTo(x, y);
            else {
                drawer.lineTo(x, y);
            }
        }

        drawer.close();
    }

    DrawPolygon(vertices, vertexCount, color) {
        this._applyStrokeColor(color);
        this._DrawPolygon(vertices, vertexCount);
        this._drawer.stroke();
    }

    DrawSolidPolygon(vertices, vertexCount, color) {
        this._applyFillColor(color);
        this._DrawPolygon(vertices, vertexCount);
        this._drawer.fill();
        this._drawer.stroke();
    }

    _DrawCircle(center, radius) {
        let p = this._xf.p;
        this._drawer.circle((center.x + p.x) * PTM_RATIO, (center.y + p.y) * PTM_RATIO, radius * PTM_RATIO);
    }

    DrawCircle(center, radius, color) {
        this._applyStrokeColor(color);
        this._DrawCircle(center, radius);
        this._drawer.stroke();
    }

    DrawSolidCircle(center, radius, axis, color) {
        this._applyFillColor(color);
        this._DrawCircle(center, radius);
        this._drawer.fill();
    }

    DrawSegment(p1, p2, color) {
        let drawer = this._drawer;

        if (p1.x === p2.x && p1.y === p2.y) {
            this._applyFillColor(color);
            this._DrawCircle(p1, 2 / PTM_RATIO);
            drawer.fill();
            return;
        }
        this._applyStrokeColor(color);

        box2d.b2Transform.MulXV(this._xf, p1, _tmp_vec2);
        drawer.moveTo(_tmp_vec2.x * PTM_RATIO, _tmp_vec2.y * PTM_RATIO);
        box2d.b2Transform.MulXV(this._xf, p2, _tmp_vec2);
        drawer.lineTo(_tmp_vec2.x * PTM_RATIO, _tmp_vec2.y * PTM_RATIO);
        drawer.stroke();
    }

    DrawTransform(xf) {
        let drawer = this._drawer;

        drawer.strokeColor = RED_COLOR;

        _tmp_vec2.x = _tmp_vec2.y = 0;
        box2d.b2Transform.MulXV(xf, _tmp_vec2, _tmp_vec2);
        drawer.moveTo(_tmp_vec2.x * PTM_RATIO, _tmp_vec2.y * PTM_RATIO);

        _tmp_vec2.x = 1;
        _tmp_vec2.y = 0;
        box2d.b2Transform.MulXV(xf, _tmp_vec2, _tmp_vec2);
        drawer.lineTo(_tmp_vec2.x * PTM_RATIO, _tmp_vec2.y * PTM_RATIO);

        drawer.stroke();

        drawer.strokeColor = GREEN_COLOR;

        _tmp_vec2.x = _tmp_vec2.y = 0;
        box2d.b2Transform.MulXV(xf, _tmp_vec2, _tmp_vec2);
        drawer.moveTo(_tmp_vec2.x * PTM_RATIO, _tmp_vec2.y * PTM_RATIO);

        _tmp_vec2.x = 0;
        _tmp_vec2.y = 1;
        box2d.b2Transform.MulXV(xf, _tmp_vec2, _tmp_vec2);
        drawer.lineTo(_tmp_vec2.x * PTM_RATIO, _tmp_vec2.y * PTM_RATIO);

        drawer.stroke();
    }

    DrawPoint(center, radius, color) {}

    _applyStrokeColor(color) {
        let strokeColor = this._drawer.strokeColor;
        strokeColor.r = color.r * 255;
        strokeColor.g = color.g * 255;
        strokeColor.b = color.b * 255;
        strokeColor.a = 150;
        this._drawer.strokeColor = strokeColor;
    }
    _applyFillColor(color) {
        let fillColor = this._drawer.fillColor;
        fillColor.r = color.r * 255;
        fillColor.g = color.g * 255;
        fillColor.b = color.b * 255;
        fillColor.a = 150;

        this._drawer.fillColor = fillColor;
    }

    PushTransform(xf) {
        this._xf = xf;
    }

    PopTransform() {
        this._xf = this._dxf;
    }
}

let tempb2Vec21 = new box2d.b2Vec2();
let tempb2Vec22 = new box2d.b2Vec2();
let tempVec21 = { x: 0, y: 0 };

export class RigidBody {
    public host: Host = null;

    private _inited = false;
    private _b2Body = null;

    /**
     * 是否启用接触接听器。
     * 当 collider 产生碰撞时，只有开启了接触接听器才会调用相应的回调函数
     * @property {Boolean} enabledContactListener
     * @default false
     */
    public enabledContactListener = false;

    /**
     * 碰撞回调。
     * 如果你的脚本中实现了这个函数，那么它将会在两个碰撞体开始接触时被调用。
     * @method onBeginContact
     * @param {PhysicsContact} contact - contact information
     * @param {PhysicsCollider} selfCollider - the collider belong to this rigidbody
     * @param {PhysicsCollider} otherCollider - the collider belong to another rigidbody
     */
    /**
     * 碰撞回调。
     * 如果你的脚本中实现了这个函数，那么它将会在两个碰撞体停止接触时被调用。
     * @method onEndContact
     * @param {PhysicsContact} contact - contact information
     * @param {PhysicsCollider} selfCollider - the collider belong to this rigidbody
     * @param {PhysicsCollider} otherCollider - the collider belong to another rigidbody
     */
    /**
     * 碰撞回调。
     * 如果你的脚本中实现了这个函数，那么它将会在接触更新时被调用。
     * 你可以在接触被处理前根据他包含的信息作出相应的处理，比如将这个接触禁用掉。
     * 注意：回调只会为醒着的刚体调用。
     * 注意：接触点为零的时候也有可能被调用。
     * 注意：感知体(sensor)的回调不会被调用。
     * @method onPreSolve
     * @param {PhysicsContact} contact - contact information
     * @param {PhysicsCollider} selfCollider - the collider belong to this rigidbody
     * @param {PhysicsCollider} otherCollider - the collider belong to another rigidbody
     */
    /**
     * 碰撞回调。
     * 如果你的脚本中实现了这个函数，那么它将会在接触更新完后被调用。
     * 你可以在这个回调中从接触信息中获取到冲量信息。
     * @method onPostSolve
     * @param {PhysicsContact} contact - contact information
     * @param {PhysicsCollider} selfCollider - the collider belong to this rigidbody
     * @param {PhysicsCollider} otherCollider - the collider belong to another rigidbody
     */

    /**
     * 这个刚体是否是一个快速移动的刚体，并且需要禁止穿过其他快速移动的刚体？
     * 需要注意的是 :
     *  - 所有刚体都被禁止从 运动刚体 和 静态刚体 中穿过。此选项只关注于 动态刚体。
     *  - 应该尽量少的使用此选项，因为它会增加程序处理时间。
     * @property {Boolean} bullet
     * @default false
     */
    public bullet = false;

    /**
     * 刚体类型： Static, Kinematic, Dynamic or Animated.
     * @property {RigidBodyType} type
     * @default RigidBodyType.Dynamic
     */
    public get type() {
        return this._b2Body.GetType();
    }

    public set type(value: RigidBodyType) {
        if (value === RigidBodyType.Animated) {
            this._b2Body.SetType(RigidBodyType.Kinematic);
        } else {
            this._b2Body.SetType(value);
        }
    }

    /**
     * 如果此刚体永远都不应该进入睡眠，那么设置这个属性为 false。
     * 需要注意这将使 CPU 占用率提高。
     * @property {Boolean} allowSleep
     * @default true
     */
    public get allowSleep() {
        return this._b2Body.IsSleepingAllowed();
    }

    public set allowSleep(value) {
        this._b2Body.SetSleepingAllowed(value);
    }

    /**
     * 缩放应用在此刚体上的重力值
     * @property {Number} gravityScale
     * @default 1
     */
    public get gravityScale() {
        return this._b2Body.GetGravityScale();
    }

    public set gravityScale(value) {
        this._b2Body.SetGravityScale(value);
    }

    /**
     * Linear damping 用于衰减刚体的线性速度。衰减系数可以大于 1，但是当衰减系数比较大的时候，衰减的效果会变得比较敏感。
     * @property {Number} linearDamping
     * @default 0
     */
    public get linearDamping() {
        return this._b2Body.GetLinearDamping();
    }

    public set linearDamping(value) {
        this._b2Body.SetLinearDamping(value);
    }

    /**
     * Angular damping 用于衰减刚体的角速度。衰减系数可以大于 1，但是当衰减系数比较大的时候，衰减的效果会变得比较敏感。
     * @property {Number} angularDamping
     * @default 0
     */
    public get angularDamping() {
        return this._b2Body.GetAngularDamping();
    }

    public set angularDamping(value) {
        this._b2Body.SetAngularDamping(value);
    }

    /**
     * 刚体在世界坐标下的线性速度
     * @property {Vec2} linearVelocity
     */
    public get linearVelocity() {
        let velocity = this._b2Body.GetLinearVelocity();
        tempVec21.x = velocity.x * PTM_RATIO;
        tempVec21.y = velocity.y * PTM_RATIO;
        return tempVec21;
    }

    public set linearVelocity(value) {
        let temp = this._b2Body.m_linearVelocity;
        temp.Set(value.x / PTM_RATIO, value.y / PTM_RATIO);
        this._b2Body.SetLinearVelocity(temp);
    }

    public get linearVelocityX() {
        let velocity = this._b2Body.GetLinearVelocity();
        return velocity.x * PTM_RATIO;
    }

    public set linearVelocityX(value) {
        this._b2Body.SetLinearVelocityX(value / PTM_RATIO);
    }

    public get linearVelocityY() {
        let velocity = this._b2Body.GetLinearVelocity();
        return velocity.y * PTM_RATIO;
    }

    public set linearVelocityY(value) {
        this._b2Body.SetLinearVelocityY(value / PTM_RATIO);
    }

    /**
     * 刚体的角速度
     * @property {Number} angularVelocity
     * @default 0
     */
    public get angularVelocity() {
        return this._b2Body.GetAngularVelocity() * PHYSICS_ANGLE_TO_ANGLE;
    }

    public set angularVelocity(value) {
        this._b2Body.SetAngularVelocity(value * ANGLE_TO_PHYSICS_ANGLE);
    }

    /**
     * 是否禁止此刚体进行旋转
     * @property {Boolean} fixedRotation
     * @default false
     */
    public get fixedRotation() {
        return this._b2Body.IsFixedRotation();
    }

    public set fixedRotation(value) {
        this._b2Body.SetFixedRotation(value);
    }

    /**
     * 设置刚体的睡眠状态。 睡眠的刚体具有非常低的 CPU 成本。（当刚体被碰撞到时，如果刚体处于睡眠状态，它会立即被唤醒）
     * @property {Boolean} awake
     * @default false
     */
    public get awake() {
        return this._b2Body.IsAwake();
    }

    public set awake(value) {
        if (this._b2Body) {
            this._b2Body.SetAwake(value);
        }
    }

    /**
     * 是否在初始化时唤醒此刚体
     * @property {Boolean} awakeOnLoad
     * @default true
     */
    public awakeOnLoad = true;

    /**
     * 设置刚体的激活状态。一个非激活状态下的刚体是不会被模拟和碰撞的，不管它是否处于睡眠状态下。
     * 如果刚体处于激活状态下，所有夹具会被添加到 粗测阶段（broad-phase）。
     * 如果刚体处于非激活状态下，所有夹具会被从 粗测阶段（broad-phase）中移除。
     * 在非激活状态下的夹具不会参与到碰撞，射线，或者查找中
     * 链接到非激活状态下刚体的关节也是非激活的。
     * @property {Boolean} active
     * @default true
     */
    public get active() {
        return this._b2Body.IsActive();
    }

    public set active(value) {
        if (this._b2Body) {
            this._b2Body.SetActive(value);
        }
    }

    /**
     * 将一个给定的世界坐标系下的点转换为刚体本地坐标系下的点
     * @method getLocalPoint
     * @param {Vec2} worldPoint - a point in world coordinates.
     * @param {Vec2} out - optional, the receiving point
     * @return {Vec2} the corresponding local point relative to the body's origin.
     */
    getLocalPoint(worldPoint, out) {
        out = out || { x: 0, y: 0 };
        if (this._b2Body) {
            tempb2Vec21.Set(worldPoint.x / PTM_RATIO, worldPoint.y / PTM_RATIO);
            let pos = this._b2Body.GetLocalPoint(tempb2Vec21, out);
            out.x = pos.x * PTM_RATIO;
            out.y = pos.y * PTM_RATIO;
        }
        return out;
    }

    /**
     * 将一个给定的刚体本地坐标系下的点转换为世界坐标系下的点
     * @method getWorldPoint
     * @param {Vec2} localPoint - a point in local coordinates.
     * @param {Vec2} out - optional, the receiving point
     * @return {Vec2} the same point expressed in world coordinates.
     */
    getWorldPoint(localPoint, out) {
        out = out || { x: 0, y: 0 };
        if (this._b2Body) {
            tempb2Vec21.Set(localPoint.x / PTM_RATIO, localPoint.y / PTM_RATIO);
            let pos = this._b2Body.GetWorldPoint(tempb2Vec21, out);
            out.x = pos.x * PTM_RATIO;
            out.y = pos.y * PTM_RATIO;
        }
        return out;
    }

    /**
     * 将一个给定的世界坐标系下的向量转换为刚体本地坐标系下的向量
     * @method getWorldVector
     * @param {Vec2} localVector - a vector in world coordinates.
     * @param {Vec2} out - optional, the receiving vector
     * @return {Vec2} the same vector expressed in local coordinates.
     */
    getWorldVector(localVector, out) {
        out = out || { x: 0, y: 0 };
        if (this._b2Body) {
            tempb2Vec21.Set(localVector.x / PTM_RATIO, localVector.y / PTM_RATIO);
            let vector = this._b2Body.GetWorldVector(tempb2Vec21, out);
            out.x = vector.x * PTM_RATIO;
            out.y = vector.y * PTM_RATIO;
        }
        return out;
    }

    /**
     * 将一个给定的世界坐标系下的点转换为刚体本地坐标系下的点
     * @method getLocalVector
     * @param {Vec2} worldVector - a vector in world coordinates.
     * @param {Vec2} out - optional, the receiving vector
     * @return {Vec2} the corresponding local vector relative to the body's origin.
     */
    getLocalVector(worldVector, out) {
        out = out || { x: 0, y: 0 };
        if (this._b2Body) {
            tempb2Vec21.Set(worldVector.x / PTM_RATIO, worldVector.y / PTM_RATIO);
            let vector = this._b2Body.GetLocalVector(tempb2Vec21, out);
            out.x = vector.x * PTM_RATIO;
            out.y = vector.y * PTM_RATIO;
        }
        return out;
    }

    /**
     * 获取刚体世界坐标系下的原点值
     * @method getWorldPosition
     * @param {Vec2} out - optional, the receiving point
     * @return {Vec2} the world position of the body's origin.
     */
    getWorldPosition(out) {
        out = out || { x: 0, y: 0 };
        if (this._b2Body) {
            let pos = this._b2Body.GetPosition();
            out.x = pos.x * PTM_RATIO;
            out.y = pos.y * PTM_RATIO;
        }
        return out;
    }

    getWorldPositionX() {
        if (this._b2Body) {
            let pos = this._b2Body.GetPosition();
            return pos.x * PTM_RATIO;
        }
        return 0;
    }

    getWorldPositionY() {
        if (this._b2Body) {
            let pos = this._b2Body.GetPosition();
            return pos.y * PTM_RATIO;
        }
        return 0;
    }

    /**
     * 获取刚体世界坐标系下的旋转值。
     * @method getWorldRotation
     * @return {Number} the current world rotation angle.
     */
    getWorldRotation() {
        if (this._b2Body) {
            return this._b2Body.GetAngle() * PHYSICS_ANGLE_TO_ANGLE;
        }
        return 0;
    }

    /**
     * 获取刚体本地坐标系下的质心
     * @method getLocalCenter
     * @return {Vec2} the local position of the center of mass.
     */
    getLocalCenter(out) {
        out = out || { x: 0, y: 0 };
        if (this._b2Body) {
            let pos = this._b2Body.GetLocalCenter();
            out.x = pos.x * PTM_RATIO;
            out.y = pos.y * PTM_RATIO;
        }
        return out;
    }

    /**
     * 获取刚体世界坐标系下的质心
     * @method getWorldCenter
     * @return {Vec2} the world position of the center of mass.
     */
    getWorldCenter(out) {
        out = out || { x: 0, y: 0 };
        if (this._b2Body) {
            let pos = this._b2Body.GetWorldCenter();
            out.x = pos.x * PTM_RATIO;
            out.y = pos.y * PTM_RATIO;
        }
        return out;
    }

    /**
     * 获取刚体上指定点的线性速度
     * @method getLinearVelocityFromWorldPoint
     * @param {Vec2} worldPoint - a point in world coordinates.
     * @param {Vec2} out - optional, the receiving point
     * @return {Vec2} the world velocity of a point.
     */
    getLinearVelocityFromWorldPoint(worldPoint, out) {
        out = out || { x: 0, y: 0 };
        if (this._b2Body) {
            tempb2Vec21.Set(worldPoint.x / PTM_RATIO, worldPoint.y / PTM_RATIO);
            let velocity = this._b2Body.GetLinearVelocityFromWorldPoint(tempb2Vec21, out);
            out.x = velocity.x * PTM_RATIO;
            out.y = velocity.y * PTM_RATIO;
        }
        return out;
    }

    /**
     * 获取刚体的质量。
     * @method getMass
     * @return {Number} the total mass of the body.
     */
    getMass() {
        return this._b2Body ? this._b2Body.GetMass() : 0;
    }

    /**
     * 获取刚体本地坐标系下原点的旋转惯性
     * @method getInertia
     * @return {Number} the rotational inertia, usually in kg-m^2.
     */
    getInertia() {
        return this._b2Body ? this._b2Body.GetInertia() * PTM_RATIO * PTM_RATIO : 0;
    }

    /**
     * 获取链接到此刚体的所有关节
     * @method getJointList
     * @return {[Joint]} the joint list.
     */
    getJointList() {
        if (!this._b2Body) return [];

        let joints = [];

        let list = this._b2Body.GetJointList();
        if (!list) return [];

        joints.push(list.joint._joint);

        // find prev joint
        let prev = list.prev;
        while (prev) {
            joints.push(prev.joint._joint);
            prev = prev.prev;
        }

        // find next joint
        let next = list.next;
        while (next) {
            joints.push(next.joint._joint);
            next = next.next;
        }

        return joints;
    }

    /**
     * 施加一个力到刚体上的一个点。如果力没有施加到刚体的质心上，还会产生一个扭矩并且影响到角速度。
     * @method applyForce
     * @param {Vec2} force - the world force vector.
     * @param {Vec2} point - the world position.
     * @param {Boolean} wake - also wake up the body.
     */
    applyForce(force, point, wake) {
        if (this._b2Body) {
            tempb2Vec21.Set(force.x / PTM_RATIO, force.y / PTM_RATIO);
            tempb2Vec22.Set(point.x / PTM_RATIO, point.y / PTM_RATIO);
            this._b2Body.ApplyForce(tempb2Vec21, tempb2Vec22, wake);
        }
    }

    /**
     * 施加一个力到刚体上的质心上。
     * @method applyForceToCenter
     * @param {Vec2} force - the world force vector.
     * @param {Boolean} wake - also wake up the body.
     */
    applyForceToCenter(force, wake) {
        if (this._b2Body) {
            tempb2Vec21.Set(force.x / PTM_RATIO, force.y / PTM_RATIO);
            this._b2Body.ApplyForceToCenter(tempb2Vec21, wake);
        }
    }

    /**
     * 施加一个扭矩力，将影响刚体的角速度
     * @method applyTorque
     * @param {Number} torque - about the z-axis (out of the screen), usually in N-m.
     * @param {Boolean} wake - also wake up the body
     */
    applyTorque(torque, wake) {
        if (this._b2Body) {
            this._b2Body.ApplyTorque(torque / PTM_RATIO, wake);
        }
    }

    /**
     * 施加冲量到刚体上的一个点，将立即改变刚体的线性速度。
     * 如果冲量施加到的点不是刚体的质心，那么将产生一个扭矩并影响刚体的角速度。
     * @method applyLinearImpulse
     * @param {Vec2} impulse - the world impulse vector, usually in N-seconds or kg-m/s.
     * @param {Vec2} point - the world position
     * @param {Boolean} wake - alse wake up the body
     */
    applyLinearImpulse(impulse, point, wake) {
        if (this._b2Body) {
            tempb2Vec21.Set(impulse.x / PTM_RATIO, impulse.y / PTM_RATIO);
            tempb2Vec22.Set(point.x / PTM_RATIO, point.y / PTM_RATIO);
            this._b2Body.ApplyLinearImpulse(tempb2Vec21, tempb2Vec22, wake);
        }
    }

    /**
     * 施加一个角速度冲量。
     * @method applyAngularImpulse
     * @param {Number} impulse - the angular impulse in units of kg*m*m/s
     * @param {Boolean} wake - also wake up the body
     */
    applyAngularImpulse(impulse, wake) {
        if (this._b2Body) {
            this._b2Body.ApplyAngularImpulse(impulse / PTM_RATIO / PTM_RATIO, wake);
        }
    }

    /**
     * 同步节点的世界坐标到 box2d 刚体的坐标上。
     * 如果 enableAnimated 是 true，并且刚体的类型是 Animated ，那么将设置刚体的线性速度来代替直接设置刚体的位置。
     * @method syncPosition
     * @param {Boolean} enableAnimated
     */
    syncPosition(enableAnimated, x, y) {
        let b2body = this._b2Body;
        if (!b2body) return;

        let temp;
        if (this.type === RigidBodyType.Animated) {
            temp = b2body.GetLinearVelocity();
        } else {
            temp = b2body.GetPosition();
        }

        temp.x = x / PTM_RATIO;
        temp.y = y / PTM_RATIO;

        if (this.type === RigidBodyType.Animated && enableAnimated) {
            let b2Pos = b2body.GetPosition();

            temp.x = (temp.x - b2Pos.x) * PhysicsManager.TIMESTEP;
            temp.y = (temp.y - b2Pos.y) * PhysicsManager.TIMESTEP;

            b2body.SetAwake(true);
            b2body.SetLinearVelocity(temp);
        } else {
            b2body.SetTransformVec(temp, b2body.GetAngle());
        }
    }

    syncPositionX(enableAnimated, x) {
        let b2body = this._b2Body;
        if (!b2body) return;

        let temp;
        if (this.type === RigidBodyType.Animated) {
            temp = b2body.GetLinearVelocity();
        } else {
            temp = b2body.GetPosition();
        }

        temp.x = x / PTM_RATIO;

        if (this.type === RigidBodyType.Animated && enableAnimated) {
            let b2Pos = b2body.GetPosition();

            temp.x = (temp.x - b2Pos.x) * PhysicsManager.TIMESTEP;

            b2body.SetAwake(true);
            b2body.SetLinearVelocity(temp);
        } else {
            b2body.SetTransformVec(temp, b2body.GetAngle());
        }
    }

    syncPositionY(enableAnimated, y) {
        let b2body = this._b2Body;
        if (!b2body) return;

        let temp;
        if (this.type === RigidBodyType.Animated) {
            temp = b2body.GetLinearVelocity();
        } else {
            temp = b2body.GetPosition();
        }

        temp.y = y / PTM_RATIO;

        if (this.type === RigidBodyType.Animated && enableAnimated) {
            let b2Pos = b2body.GetPosition();

            temp.y = (temp.y - b2Pos.y) * PhysicsManager.TIMESTEP;

            b2body.SetAwake(true);
            b2body.SetLinearVelocity(temp);
        } else {
            b2body.SetTransformVec(temp, b2body.GetAngle());
        }
    }

    /**
     * 同步节点的世界旋转角度值到 box2d 刚体的旋转值上。
     * 如果 enableAnimated 是 true，并且刚体的类型是 Animated ，那么将设置刚体的角速度来代替直接设置刚体的角度。
     * @method syncRotation
     * @param {Boolean} enableAnimated
     */
    syncRotation(enableAnimated, angle) {
        let b2body = this._b2Body;
        if (!b2body) return;

        let rotation = ANGLE_TO_PHYSICS_ANGLE * -angle;
        if (this.type === RigidBodyType.Animated && enableAnimated) {
            let b2Rotation = b2body.GetAngle();
            b2body.SetAwake(true);
            b2body.SetAngularVelocity((rotation - b2Rotation) * PhysicsManager.TIMESTEP);
        } else {
            b2body.SetTransformVec(b2body.GetPosition(), rotation);
        }
    }

    resetVelocity() {
        let b2body = this._b2Body;
        if (!b2body) return;

        let temp = b2body.m_linearVelocity;
        temp.Set(0, 0);

        b2body.SetLinearVelocity(temp);
        b2body.SetAngularVelocity(0);
    }

    init(
        host: Host = null,
        x: number = 0,
        y: number = 0,
        angle: number = 0,
        enabledContactListener: boolean = false,
        bullet: boolean = false,
        type: RigidBodyType = RigidBodyType.Dynamic,
        allowSleep: boolean = true,
        gravityScale: number = 1,
        linearDamping: number = 0,
        angularDamping: number = 0,
        linearVelocity = { x: 0, y: 0 },
        angularVelocity: number = 0,
        fixedRotation: boolean = true,
        awakeOnLoad: boolean = true
    ) {
        PhysicsManager.getPhysicsManager()._pushDelayEvent(this, "__init", [
            host,
            x,
            y,
            angle,
            enabledContactListener,
            bullet,
            type,
            allowSleep,
            gravityScale,
            linearDamping,
            angularDamping,
            linearVelocity,
            angularVelocity,
            fixedRotation,
            awakeOnLoad,
        ]);
    }

    destroy() {
        PhysicsManager.getPhysicsManager()._pushDelayEvent(this, "__destroy", []);
    }

    __init(
        host,
        x,
        y,
        angle,
        enabledContactListener,
        bullet,
        type,
        allowSleep,
        gravityScale,
        linearDamping,
        angularDamping,
        linearVelocity,
        angularVelocity,
        fixedRotation,
        awakeOnLoad
    ) {
        if (this._inited) return;

        this.host = host;
        this.enabledContactListener = enabledContactListener;
        this.bullet = bullet;
        this.awakeOnLoad = awakeOnLoad;

        let bodyDef = new box2d.b2BodyDef();

        if (type === RigidBodyType.Animated) {
            bodyDef.type = RigidBodyType.Kinematic;
        } else {
            bodyDef.type = type;
        }

        bodyDef.allowSleep = allowSleep;
        bodyDef.gravityScale = gravityScale;
        bodyDef.linearDamping = linearDamping;
        bodyDef.angularDamping = angularDamping;

        bodyDef.linearVelocity = new box2d.b2Vec2(linearVelocity.x / PTM_RATIO, linearVelocity.y / PTM_RATIO);

        bodyDef.angularVelocity = angularVelocity * ANGLE_TO_PHYSICS_ANGLE;

        bodyDef.fixedRotation = fixedRotation;
        bodyDef.bullet = this.bullet;

        bodyDef.position = new box2d.b2Vec2(x / PTM_RATIO, y / PTM_RATIO);
        bodyDef.angle = ANGLE_TO_PHYSICS_ANGLE * -angle;
        bodyDef.awake = this.awakeOnLoad;
        PhysicsManager.getPhysicsManager()._addBody(this, bodyDef);

        this._inited = true;
    }

    __destroy() {
        if (!this._inited) return;

        PhysicsManager.getPhysicsManager()._removeBody(this);

        this._inited = false;
        this.host = null;
    }

    _getBody() {
        return this._b2Body;
    }
}

export class PhysicsCollider {
    private _fixtures = [];
    private _shapes = [];
    private _inited = false;
    private _rect = { x: 0, y: 0, width: 0, height: 0 };

    /**
     * 密度
     * @property {Number} density
     * @default 1
     */
    public density = 1;

    /**
     * 一个传感器类型的碰撞体会产生碰撞回调，但是不会发生物理碰撞效果。
     * @property {Boolean} sensor
     * @default false
     */
    public sensor = false;

    /**
     * 摩擦系数，取值一般在 [0, 1] 之间
     * @property {Number} friction
     * @default 0.2
     */
    public friction = 0.2;

    /**
     * 弹性系数，取值一般在 [0, 1]之间
     * @property {Number} restitution
     * @default 0
     */
    public restitution = 0;

    /**
     * 碰撞体会在初始化时查找节点上是否存在刚体，如果查找成功则赋值到这个属性上。
     * @property {RigidBody} body
     * @default null
     */
    public body: RigidBody = null;

    _getFixtureIndex(fixture) {
        return this._fixtures.indexOf(fixture);
    }

    init(scaleX, scaleY, groupIndex) {
        PhysicsManager.getPhysicsManager()._pushDelayEvent(this, "__init", [scaleX, scaleY, groupIndex]);
    }

    destroy() {
        PhysicsManager.getPhysicsManager()._pushDelayEvent(this, "__destroy", []);
    }

    __init(scaleX, scaleY, groupIndex) {
        if (this._inited) return;

        let body = this.body;
        if (!body) return;

        let innerBody = body._getBody();
        if (!innerBody) return;

        let shapes = scaleX === 0 && scaleY === 0 ? [] : this._createShape(scaleX, scaleY);

        if (!(shapes instanceof Array)) {
            shapes = [shapes];
        }

        //分组索引
        let categoryBits = 1 << groupIndex;
        let maskBits = 0;

        let bits = PhysicsManager.CollisionMatrix[groupIndex];
        for (let i = 0; i < bits.length; i++) {
            if (!bits[i]) continue;
            maskBits |= 1 << i;
        }

        let filter = {
            categoryBits: categoryBits,
            maskBits: maskBits,
            groupIndex: 0,
        };

        for (let i = 0; i < shapes.length; i++) {
            let shape = shapes[i];

            let fixDef = new box2d.b2FixtureDef();
            fixDef.density = this.density;
            fixDef.isSensor = this.sensor;
            fixDef.friction = this.friction;
            fixDef.restitution = this.restitution;
            fixDef.shape = shape;

            fixDef.filter = filter;

            let fixture = innerBody.CreateFixture(fixDef);
            fixture.collider = this;

            if (body.enabledContactListener) {
                PhysicsManager.getPhysicsManager()._registerContactFixture(fixture);
            }

            this._shapes.push(shape);
            this._fixtures.push(fixture);
        }

        this.body = body;

        this._inited = true;
    }

    __destroy() {
        if (!this._inited) return;

        let fixtures = this._fixtures;
        let body = this.body._getBody();

        for (let i = fixtures.length - 1; i >= 0; i--) {
            let fixture = fixtures[i];
            fixture.collider = null;

            PhysicsManager.getPhysicsManager()._unregisterContactFixture(fixture);

            if (body) {
                body.DestroyFixture(fixture);
            }
        }

        this.body = null;

        this._fixtures.length = 0;
        this._shapes.length = 0;
        this._inited = false;
    }

    _createShape(scaleX, scaleY) {}

    /**
     * !#en
     * Apply current changes to collider, this will regenerate inner box2d fixtures.
     * !#zh
     * 应用当前 collider 中的修改，调用此函数会重新生成内部 box2d 的夹具。
     * @method apply
     */
    apply(scaleX, scaleY, groupIndex) {
        this.destroy();
        this.init(scaleX, scaleY, groupIndex);
    }

    /**
     * !#en
     * Get the world aabb of the collider
     * !#zh
     * 获取碰撞体的世界坐标系下的包围盒
     * @method getAABB
     */
    getAABB(_fixtures) {
        let MAX = 10e6;

        let minX = MAX,
            minY = MAX;
        let maxX = -MAX,
            maxY = -MAX;

        let fixtures = _fixtures || this._fixtures;
        for (let i = 0; i < fixtures.length; i++) {
            let fixture = fixtures[i];

            let count = fixture.GetShape().GetChildCount();
            for (let j = 0; j < count; j++) {
                let aabb = fixture.GetAABB(j);
                if (aabb.lowerBound.x < minX) minX = aabb.lowerBound.x;
                if (aabb.lowerBound.y < minY) minY = aabb.lowerBound.y;
                if (aabb.upperBound.x > maxX) maxX = aabb.upperBound.x;
                if (aabb.upperBound.y > maxY) maxY = aabb.upperBound.y;
            }
        }

        minX *= PTM_RATIO;
        minY *= PTM_RATIO;
        maxX *= PTM_RATIO;
        maxY *= PTM_RATIO;

        let r = this._rect;
        r.x = minX;
        r.y = minY;
        r.width = maxX - minX;
        r.height = maxY - minY;

        return r;
    }
}

export class PhysicsCircleCollider extends PhysicsCollider {
    public offset = { x: 0, y: 0 };
    public radius = 50;

    _createShape(scaleX, scaleY) {
        scaleX = Math.abs(scaleX);
        scaleY = Math.abs(scaleY);
        let offsetX = (this.offset.x / PTM_RATIO) * scaleX;
        let offsetY = (this.offset.y / PTM_RATIO) * scaleY;

        let shape = new box2d.b2CircleShape();
        shape.m_radius = (this.radius / PTM_RATIO) * scaleX;
        shape.m_p = new box2d.b2Vec2(offsetX, offsetY);

        return shape;
    }
}

export class PhysicsBoxCollider extends PhysicsCollider {
    public offset = { x: 0, y: 0 };
    public size = { width: 100, height: 100 };

    _createShape(scaleX, scaleY) {
        scaleX = Math.abs(scaleX);
        scaleY = Math.abs(scaleY);
        let width = (this.size.width / 2 / PTM_RATIO) * scaleX;
        let height = (this.size.height / 2 / PTM_RATIO) * scaleY;
        let offsetX = (this.offset.x / PTM_RATIO) * scaleX;
        let offsetY = (this.offset.y / PTM_RATIO) * scaleY;

        let shape = new box2d.b2PolygonShape();
        shape.SetAsBox(width, height, new box2d.b2Vec2(offsetX, offsetY), 0);
        return shape;
    }
}

// http://answers.unity3d.com/questions/977416/2d-polygon-convex-decomposition-code.html

/// <summary>
/// This class is took from the "FarseerUnity" physics engine, which uses Mark Bayazit's decomposition algorithm.
/// I also have to make it work with self-intersecting polygons, so I'll use another different algorithm to decompose a self-intersecting polygon into several simple polygons,
/// and then I would decompose each of them into convex polygons.
/// </summary>

//From phed rev 36

/// <summary>
/// Convex decomposition algorithm created by Mark Bayazit (http://mnbayazit.com/)
/// For more information about this algorithm, see http://mnbayazit.com/406/bayazit
/// </summary>

function At(i, vertices) {
    let s = vertices.length;
    return vertices[i < 0 ? s - (-i % s) : i % s];
}

function Copy(i, j, vertices) {
    let p = [];
    while (j < i) j += vertices.length;
    //p.reserve(j - i + 1);
    for (; i <= j; ++i) {
        p.push(At(i, vertices));
    }
    return p;
}

/// <summary>
/// Decompose the polygon into several smaller non-concave polygon.
/// If the polygon is already convex, it will return the original polygon, unless it is over Settings.MaxPolygonVertices.
/// Precondition: Counter Clockwise polygon
/// </summary>
/// <param name="vertices"></param>
/// <returns></returns>
function ConvexPartition(vertices) {
    //We force it to CCW as it is a precondition in this algorithm.
    ForceCounterClockWise(vertices);

    let list = [];
    let d, lowerDist, upperDist;
    let p;
    let lowerInt = cc.v2();
    let upperInt = cc.v2(); // intersection points
    let lowerIndex = 0,
        upperIndex = 0;
    let lowerPoly, upperPoly;

    for (let i = 0; i < vertices.length; ++i) {
        if (Reflex(i, vertices)) {
            lowerDist = upperDist = 10e7; // std::numeric_limits<qreal>::max();
            for (let j = 0; j < vertices.length; ++j) {
                // if line intersects with an edge
                if (Left(At(i - 1, vertices), At(i, vertices), At(j, vertices)) && RightOn(At(i - 1, vertices), At(i, vertices), At(j - 1, vertices))) {
                    // find the povar of intersection
                    p = LineIntersect(At(i - 1, vertices), At(i, vertices), At(j, vertices), At(j - 1, vertices));
                    if (Right(At(i + 1, vertices), At(i, vertices), p)) {
                        // make sure it's inside the poly
                        d = SquareDist(At(i, vertices), p);
                        if (d < lowerDist) {
                            // keep only the closest intersection
                            lowerDist = d;
                            lowerInt = p;
                            lowerIndex = j;
                        }
                    }
                }

                if (Left(At(i + 1, vertices), At(i, vertices), At(j + 1, vertices)) && RightOn(At(i + 1, vertices), At(i, vertices), At(j, vertices))) {
                    p = LineIntersect(At(i + 1, vertices), At(i, vertices), At(j, vertices), At(j + 1, vertices));
                    if (Left(At(i - 1, vertices), At(i, vertices), p)) {
                        d = SquareDist(At(i, vertices), p);
                        if (d < upperDist) {
                            upperDist = d;
                            upperIndex = j;
                            upperInt = p;
                        }
                    }
                }
            }

            // if there are no vertices to connect to, choose a povar in the middle
            if (lowerIndex == (upperIndex + 1) % vertices.length) {
                let sp = lowerInt.add(upperInt).div(2);

                lowerPoly = Copy(i, upperIndex, vertices);
                lowerPoly.push(sp);
                upperPoly = Copy(lowerIndex, i, vertices);
                upperPoly.push(sp);
            } else {
                let highestScore = 0,
                    bestIndex = lowerIndex;

                while (upperIndex < lowerIndex) {
                    upperIndex += vertices.length;
                }

                for (let j = lowerIndex; j <= upperIndex; ++j) {
                    if (CanSee(i, j, vertices)) {
                        let score = 1 / (SquareDist(At(i, vertices), At(j, vertices)) + 1);
                        if (Reflex(j, vertices)) {
                            if (RightOn(At(j - 1, vertices), At(j, vertices), At(i, vertices)) && LeftOn(At(j + 1, vertices), At(j, vertices), At(i, vertices))) {
                                score += 3;
                            } else {
                                score += 2;
                            }
                        } else {
                            score += 1;
                        }

                        if (score > highestScore) {
                            bestIndex = j;
                            highestScore = score;
                        }
                    }
                }
                lowerPoly = Copy(i, bestIndex, vertices);
                upperPoly = Copy(bestIndex, i, vertices);
            }
            list = list.concat(ConvexPartition(lowerPoly));
            list = list.concat(ConvexPartition(upperPoly));
            return list;
        }
    }

    // polygon is already convex
    list.push(vertices);

    //Remove empty vertice collections
    for (let i = list.length - 1; i >= 0; i--) {
        if (list[i].length == 0) list.splice(i, 0);
    }

    return list;
}

function CanSee(i, j, vertices) {
    if (Reflex(i, vertices)) {
        if (LeftOn(At(i, vertices), At(i - 1, vertices), At(j, vertices)) && RightOn(At(i, vertices), At(i + 1, vertices), At(j, vertices))) return false;
    } else {
        if (RightOn(At(i, vertices), At(i + 1, vertices), At(j, vertices)) || LeftOn(At(i, vertices), At(i - 1, vertices), At(j, vertices))) return false;
    }
    if (Reflex(j, vertices)) {
        if (LeftOn(At(j, vertices), At(j - 1, vertices), At(i, vertices)) && RightOn(At(j, vertices), At(j + 1, vertices), At(i, vertices))) return false;
    } else {
        if (RightOn(At(j, vertices), At(j + 1, vertices), At(i, vertices)) || LeftOn(At(j, vertices), At(j - 1, vertices), At(i, vertices))) return false;
    }

    for (let k = 0; k < vertices.length; ++k) {
        if ((k + 1) % vertices.length == i || k == i || (k + 1) % vertices.length == j || k == j) {
            continue; // ignore incident edges
        }
        let intersectionPoint = cc.v2();
        if (LineIntersect2(At(i, vertices), At(j, vertices), At(k, vertices), At(k + 1, vertices), intersectionPoint)) {
            return false;
        }
    }
    return true;
}

// precondition: ccw
function Reflex(i, vertices) {
    return Right(i, vertices, undefined);
}

function Right(a, b, c) {
    if (typeof c === "undefined") {
        let i = a,
            vertices = b;

        a = At(i - 1, vertices);
        b = At(i, vertices);
        c = At(i + 1, vertices);
    }

    return Area(a, b, c) < 0;
}

function Left(a, b, c) {
    return Area(a, b, c) > 0;
}

function LeftOn(a, b, c) {
    return Area(a, b, c) >= 0;
}

function RightOn(a, b, c) {
    return Area(a, b, c) <= 0;
}

function SquareDist(a, b) {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    return dx * dx + dy * dy;
}

//forces counter clock wise order.
function ForceCounterClockWise(vertices) {
    if (!IsCounterClockWise(vertices)) {
        vertices.reverse();
    }
}

function IsCounterClockWise(vertices) {
    //We just return true for lines
    if (vertices.length < 3) return true;

    return GetSignedArea(vertices) > 0;
}

//gets the signed area.
function GetSignedArea(vertices) {
    let i;
    let area = 0;

    for (i = 0; i < vertices.length; i++) {
        let j = (i + 1) % vertices.length;
        area += vertices[i].x * vertices[j].y;
        area -= vertices[i].y * vertices[j].x;
    }
    area /= 2;
    return area;
}

//From Mark Bayazit's convex decomposition algorithm
function LineIntersect(p1, p2, q1, q2) {
    let i = cc.v2();
    let a1 = p2.y - p1.y;
    let b1 = p1.x - p2.x;
    let c1 = a1 * p1.x + b1 * p1.y;
    let a2 = q2.y - q1.y;
    let b2 = q1.x - q2.x;
    let c2 = a2 * q1.x + b2 * q1.y;
    let det = a1 * b2 - a2 * b1;

    if (!FloatEquals(det, 0)) {
        // lines are not parallel
        i.x = (b2 * c1 - b1 * c2) / det;
        i.y = (a1 * c2 - a2 * c1) / det;
    }
    return i;
}

//from Eric Jordan's convex decomposition library, it checks if the lines a0->a1 and b0->b1 cross.
//if they do, intersectionPovar will be filled with the povar of crossing. Grazing lines should not return true.
function LineIntersect2(a0, a1, b0, b1, intersectionPoint) {
    if (a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1) return false;

    let x1 = a0.x;
    let y1 = a0.y;
    let x2 = a1.x;
    let y2 = a1.y;
    let x3 = b0.x;
    let y3 = b0.y;
    let x4 = b1.x;
    let y4 = b1.y;

    //AABB early exit
    if (Math.max(x1, x2) < Math.min(x3, x4) || Math.max(x3, x4) < Math.min(x1, x2)) return false;

    if (Math.max(y1, y2) < Math.min(y3, y4) || Math.max(y3, y4) < Math.min(y1, y2)) return false;

    let ua = (x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3);
    let ub = (x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3);
    let denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    if (Math.abs(denom) < 10e-7) {
        //Lines are too close to parallel to call
        return false;
    }
    ua /= denom;
    ub /= denom;

    if (0 < ua && ua < 1 && 0 < ub && ub < 1) {
        intersectionPoint.x = x1 + ua * (x2 - x1);
        intersectionPoint.y = y1 + ua * (y2 - y1);
        return true;
    }

    return false;
}

function FloatEquals(value1, value2) {
    return Math.abs(value1 - value2) <= 10e-7;
}

//returns a positive number if c is to the left of the line going from a to b. Positive number if povar is left, negative if povar is right, and 0 if points are collinear.</returns>
function Area(a, b, c) {
    return a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y);
}

export class PhysicsPolygonCollider extends PhysicsCollider {
    /**
     * !#zh 位置偏移量
     * @property offset
     * @type {Vec2}
     */
    public offset = { x: 0, y: 0 };

    /**
     * !#zh 多边形顶点数组
     * @property points
     * @type {Vec2[]}
     */
    public points = [
        { x: -50, y: -50 },
        { x: 50, y: -50 },
        { x: 50, y: 50 },
        { x: -50, y: 50 },
    ];

    _createShape(scaleX, scaleY) {
        let shapes = [];

        let points = this.points;

        // check if last point equal to first point
        if (points.length > 0 && points[0].x == points[points.length - 1].x && points[0].y == points[points.length - 1].y) {
            points.length -= 1;
        }

        let polys = ConvexPartition(points);
        let offset = this.offset;

        for (let i = 0; i < polys.length; i++) {
            let poly = polys[i];

            let shape = null,
                vertices = [];
            let firstVertice = null;

            for (let j = 0, l = poly.length; j < l; j++) {
                if (!shape) {
                    shape = new box2d.b2PolygonShape();
                }
                let p = poly[j];
                let x = ((p.x + offset.x) / PTM_RATIO) * scaleX;
                let y = ((p.y + offset.y) / PTM_RATIO) * scaleY;
                let v = new box2d.b2Vec2(x, y);
                vertices.push(v);

                if (!firstVertice) {
                    firstVertice = v;
                }

                if (vertices.length === box2d.b2maxPolygonVertices) {
                    shape.Set(vertices, vertices.length);
                    shapes.push(shape);

                    shape = null;

                    if (j < l - 1) {
                        vertices = [firstVertice, vertices[vertices.length - 1]];
                    }
                }
            }

            if (shape) {
                shape.Set(vertices, vertices.length);
                shapes.push(shape);
            }
        }

        return shapes;
    }
}
