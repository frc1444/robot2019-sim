package com.first1444.frc.robot2019.gdx

import com.badlogic.gdx.math.MathUtils
import com.badlogic.gdx.physics.box2d.BodyDef
import com.badlogic.gdx.physics.box2d.EdgeShape
import com.badlogic.gdx.physics.box2d.FixtureDef
import com.badlogic.gdx.physics.box2d.PolygonShape
import com.badlogic.gdx.physics.box2d.joints.RevoluteJointDef
import com.first1444.frc.robot2019.DefaultShuffleboardMap
import com.first1444.frc.robot2019.Robot
import com.first1444.frc.robot2019.ShuffleboardMap
import com.first1444.frc.robot2019.subsystems.implementations.DummyCargoIntake
import com.first1444.frc.robot2019.subsystems.implementations.DummyClimber
import com.first1444.frc.robot2019.subsystems.implementations.DummyHatchIntake
import com.first1444.frc.robot2019.subsystems.implementations.DummyLift
import com.first1444.frc.util.reportmap.ShuffleboardReportMap
import com.first1444.sim.api.RunnableCreator
import com.first1444.sim.api.Vector2
import com.first1444.sim.api.drivetrain.swerve.FourWheelSwerveDriveData
import com.first1444.sim.api.drivetrain.swerve.SwerveModule
import com.first1444.sim.api.frc.IterativeRobotRunnable
import com.first1444.sim.api.inchesToMeters
import com.first1444.sim.gdx.*
import com.first1444.sim.gdx.drivetrain.swerve.BodySwerveModule
import com.first1444.sim.gdx.entity.ActorBodyEntity
import com.first1444.sim.gdx.entity.EntityOrientation
import com.first1444.sim.gdx.implementations.deepspace2019.surroundings.VisionProvider
import com.first1444.sim.gdx.init.RobotCreator
import com.first1444.sim.gdx.init.UpdateableCreator
import com.first1444.sim.gdx.velocity.AccelerateSetPointHandler
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import me.retrodaredevil.action.Actions
import me.retrodaredevil.controller.gdx.GdxControllerPartCreator
import me.retrodaredevil.controller.gdx.IndexedControllerProvider
import me.retrodaredevil.controller.implementations.BaseStandardControllerInput
import me.retrodaredevil.controller.implementations.mappings.DefaultStandardControllerInputCreator
import me.retrodaredevil.controller.options.OptionValues
import me.retrodaredevil.controller.output.DisconnectedRumble
import java.lang.Math.toRadians

object MyRobotCreator : RobotCreator {
    override fun create(data: RobotCreator.Data, updateableData: UpdateableCreator.Data): Updateable {
        val startingPosition = Vector2(0.0, -6.6)
        val startingAngleRadians = toRadians(90.0)

        val maxVelocity = 3.35
        val wheelBase = inchesToMeters(22.75) // length
        val trackWidth = inchesToMeters(24.0)
        val entity = ActorBodyEntity(updateableData.contentStage, updateableData.worldManager.world, BodyDef().apply {
            type = BodyDef.BodyType.DynamicBody
            position.set(startingPosition)
            angle = startingAngleRadians.toFloat()
//            angle = 90 * MathUtils.degreesToRadians // start at 90 degrees to make this easy on the player. We will eventually add field centric controls
        }, listOf(FixtureDef().apply {
            restitution = .2f
            shape = PolygonShape().apply {
                setAsBox((wheelBase / 2).toFloat(), (trackWidth / 2).toFloat(), ZERO, 0.0f)
            }
            val area = wheelBase * trackWidth
            density = 1.0f / area.toFloat()
        }, FixtureDef().apply {
            isSensor = true
            shape = EdgeShape().apply {
                set(0f, 0f, .5f, 0f)
            }
        }
        ))
        val wheelBody = BodyDef().apply {
            type = BodyDef.BodyType.DynamicBody
        }
        val wheelFixture = FixtureDef().apply {
            val wheelDiameter = inchesToMeters(4.0f)
            val wheelWidth = inchesToMeters(1.0f)
            val area = wheelDiameter * wheelWidth
            shape = PolygonShape().apply {
                setAsBox(wheelDiameter / 2, wheelWidth / 2, ZERO, 0.0f) // 4 inches by 1 inch
            }
            density = 1.0f / area
        }


        val frPosition = Vector2(wheelBase / 2, -trackWidth / 2)
        val flPosition = Vector2(wheelBase / 2, trackWidth / 2)
        val rlPosition = Vector2(-wheelBase / 2, trackWidth / 2)
        val rrPosition = Vector2(-wheelBase / 2, -trackWidth / 2)

        val moduleList = ArrayList<SwerveModule>(4)
        for((moduleName, position) in listOf(
                Pair("front right", frPosition),
                Pair("front left", flPosition),
                Pair("rear left", rlPosition),
                Pair("rear right", rrPosition))){
            val wheelEntity = ActorBodyEntity(updateableData.contentStage, updateableData.worldManager.world, wheelBody, listOf(wheelFixture))
//            wheelEntity.setPosition(position)
            wheelEntity.setTransformRadians(position.rotateRadians(startingAngleRadians) + startingPosition, startingAngleRadians.toFloat())
            val joint = RevoluteJointDef().apply {
                bodyA = entity.body
                bodyB = wheelEntity.body
                localAnchorA.set(position)
                localAnchorB.set(0.0f, 0.0f)
                referenceAngle = 0.0f
            }
            updateableData.worldManager.world.createJoint(joint)
            val module = BodySwerveModule(
                    moduleName, wheelEntity.body, entity.body, maxVelocity, updateableData.clock, data.driverStation,
                    AccelerateSetPointHandler(maxVelocity.toFloat() / .5f, maxVelocity.toFloat() / .2f),
                    AccelerateSetPointHandler(MathUtils.PI2 / .5f)
            )
            moduleList.add(module)
        }
        val swerveDriveData = FourWheelSwerveDriveData(
                moduleList[0], moduleList[1], moduleList[2], moduleList[3],
                wheelBase, trackWidth
        )
        val provider = IndexedControllerProvider(0)
        val creator = GdxControllerPartCreator(provider, true)
        val joystick = BaseStandardControllerInput(DefaultStandardControllerInputCreator(), creator, OptionValues.createImmutableBooleanOptionValue(true), OptionValues.createImmutableBooleanOptionValue(false))
        val shuffleboardMap = DefaultShuffleboardMap()
        val reportMap = ShuffleboardReportMap(shuffleboardMap.debugTab.getLayout("Report Map", BuiltInLayouts.kList));
        val robotCreator = RunnableCreator.wrap {
            NetworkTableInstance.getDefault().startServer()
//            VisionProvider(entity, 2.0, updateableData.clock)
            val runnable = IterativeRobotRunnable(Robot(
                    data.driverStation, updateableData.clock,
                    shuffleboardMap,
                    joystick, GdxControllerPartCreator(IndexedControllerProvider(1)), GdxControllerPartCreator(IndexedControllerProvider(2)), DisconnectedRumble.getInstance(),
                    EntityOrientation(entity),
                    swerveDriveData,
                    DummyLift(reportMap), DummyCargoIntake(reportMap), DummyHatchIntake(reportMap), DummyClimber(reportMap),
                    Actions.createRunForeverRecyclable { }
            ), data.driverStation)
            Runnable {
                runnable.run()
                Shuffleboard.update()
            }
        }
        return UpdateableMultiplexer(listOf(
                entity,
                RobotUpdateable(robotCreator)
        ))
    }

}