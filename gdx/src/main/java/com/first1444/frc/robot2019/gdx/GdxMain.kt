package com.first1444.frc.robot2019.gdx

import com.badlogic.gdx.ApplicationListener
import com.badlogic.gdx.Gdx
import com.badlogic.gdx.scenes.scene2d.ui.Skin
import com.first1444.sim.gdx.init.*
import edu.wpi.first.networktables.NetworkTableInstance

private fun createSelectionCreator(uiSkin: Skin, changer: ScreenChanger): ScreenCreator {
    val creator = MyRobotCreator
    val exitButtonUpdateableCreator = ExitButtonCreator(Runnable {
        changer.change(createSelectionCreator(uiSkin, changer).create(changer))
        NetworkTableInstance.getDefault().close()
        // TODO also find a way to reset static shuffleboard state
        // maybe instead of using Shuffleboard.getTab, we can just create out own instances
    })
    return SelectionScreenCreator(
            uiSkin,
            FieldScreenCreator(uiSkin, UpdateableCreatorMultiplexer(listOf(
                    PracticeUpdateableCreator(creator),
                    Field2019Creator,
                    exitButtonUpdateableCreator
            ))),
            RealConfigScreenCreator(uiSkin) { _, config: RealConfig ->
                changer.change(FieldScreenCreator(uiSkin, UpdateableCreatorMultiplexer(listOf(
                        RealUpdateableCreator(uiSkin, config, creator),
                        Field2019Creator,
                        exitButtonUpdateableCreator
                ))).create(changer))
            }
    )
}

fun createScreen(): ApplicationListener {
    return SimpleGame { changer ->
        val uiSkin = Skin(Gdx.files.classpath("skins/sgx/sgx-ui.json"))
        createSelectionCreator(uiSkin, changer).create(changer)
    }
}
