/*
 * @Author: conjurer
 * @Github: https://github.com/dot123
 * @Date: 2021-03-07 19:50:29
 * @LastEditors: conjurer
 * @LastEditTime: 2021-03-07 19:52:48
 * @Description:
 */
import { PhysicsManager, RigidBodyType, RigidBody, PhysicsCircleCollider, PhysicsBoxCollider } from "./PhysicsManager";

const { ccclass, property } = cc._decorator;

@ccclass
export default class Helloworld extends cc.Component {
    @property(cc.Graphics)
    private graphics: cc.Graphics = null;

    private rect: cc.Rect = cc.rect(480 - 50, 320, 100, 100);

    onLoad() {
        // 速度更新迭代数
        PhysicsManager.VELOCITY_ITERATIONS = 1;

        // 位置迭代更新数
        PhysicsManager.POSITION_ITERATIONS = 1;

        // 指定固定的物理更新间隔时间，需要开启 enabledAccumulator 才有效。
        PhysicsManager.FIXED_TIME_STEP = 1 / 60;

        // 每次可用于更新物理系统的最大时间，需要开启 enabledAccumulator 才有效。
        PhysicsManager.MAX_ACCUMULATOR = 1 / 5;

        // 如果开启此选项，那么将会以固定的间隔时间 FIXED_TIME_STEP 来更新物理引擎，如果一个 update 的间隔时间大于 FIXED_TIME_STEP，则会对物理引擎进行多次更新。
        // 如果关闭此选项，那么将会根据设定的 frame rate 计算出一个间隔时间来更新物理引擎。
        PhysicsManager.getPhysicsManager().enabledAccumulator = false;
        PhysicsManager.getPhysicsManager().enabled = true;

        PhysicsManager.getPhysicsManager().gravity = { x: 0, y: -980 };
        PhysicsManager.CollisionMatrix = cc.game["collisionMatrix"];

        let Bits = PhysicsManager.DrawBits;
        PhysicsManager.getPhysicsManager().debugDrawFlags(Bits.e_jointBit | Bits.e_shapeBit, this.graphics);
    }

    start() {
        let body = new RigidBody();
        body.init(this, 480, 0, 0, true, false, RigidBodyType.Static);

        let boxCollider = new PhysicsBoxCollider();
        boxCollider.body = body;
        boxCollider.size.width = 960;
        boxCollider.size.height = 200;
        boxCollider.init(1, 1, 0);

        body = new RigidBody();
        body.init(this, 480, 480, 0, true, false, RigidBodyType.Dynamic, true, 1);

        let circleCollider = new PhysicsCircleCollider();
        circleCollider.body = body;
        circleCollider.radius = 50;
        circleCollider.restitution = 1;
        circleCollider.init(1, 1, 0);
    }

    clear() {
        PhysicsManager.getPhysicsManager().clear();
    }

    update(dt) {
        this.graphics.clear();
        PhysicsManager.getPhysicsManager().update(dt);
        this.TestAABB(this.rect, () => {}, true);
    }

    //画矩形
    public DrawRect(rect: cc.Rect) {
        let ctx = this.graphics;
        ctx.fillColor = cc.color(255, 0, 0, 100);
        ctx.rect(rect.x, rect.y, rect.width, rect.height);
        ctx.fill();
    }

    /**
     * 画线
     * @param startPos
     * @param endPos
     * @param results
     */
    public DrawLine(startPos, endPos, results) {
        let ctx = this.graphics;
        //画debug
        results.forEach((result) => {
            ctx.circle(result.point.x, result.point.y, 5);
        });
        ctx.fill();
        ctx.moveTo(startPos.x, startPos.y);
        ctx.lineTo(endPos.x, endPos.y);
        ctx.stroke();
    }

    public TestAABB(rect, cb, isDraw = true) {
        if (isDraw) {
            this.DrawRect(rect);
        }

        let fixtures = PhysicsManager.getPhysicsManager().testAABB(rect);
        for (let j = 0; j < fixtures.length; j++) {
            let collider = fixtures[j].collider;
            cb(collider.body.host);
        }
    }

    public RayCast(p1: cc.Vec2, p2: cc.Vec2, type: cc.RayCastType, cb) {
        let results = PhysicsManager.getPhysicsManager().rayCast(p1, p2, type);
        for (let i = 0; i < results.length; i++) {
            let collider = results[i].collider;
            if (!collider) {
                continue;
            }
            if (!cb(collider.body.host, results[i])) {
                break;
            }
        }

        //画线
        this.DrawLine(p1, p2, results);
    }

    public onBeginContact(contact, selfCollider, otherCollider) {}

    public onEndContact(contact, selfCollider, otherCollider) {}

    public onPreSolve(contact, selfCollider, otherCollider) {}

    public onPostSolve(contact, selfCollider, otherCollider) {}
}
