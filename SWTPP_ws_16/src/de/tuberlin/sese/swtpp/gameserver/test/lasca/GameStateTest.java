package de.tuberlin.sese.swtpp.gameserver.test.lasca;

import static org.junit.Assert.*;

import org.junit.Test;

import de.tuberlin.sese.swtpp.gameserver.model.lasca.GameState;
import de.tuberlin.sese.swtpp.gameserver.model.lasca.LNUtil;

public class GameStateTest {
	@Test
	public void GameStateInitTest1(){
		GameState testState = LNUtil.getGameState("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w w");
		GameState expectedState = new GameState("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w", "w");
		assertTrue("States didn't match", testState.equals(expectedState));
	}

	@Test
	public void GameStateInitTest2(){
		GameState testState = LNUtil.getGameState("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w w a3-c5");
		GameState expectedState = new GameState("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w", "w", "a3-c5");
		assertTrue("States didn't match", testState.equals(expectedState));
	}
	
	@Test
	public void GameStateToStringTest1(){
		GameState testState = LNUtil.getGameState("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w w");
		String expectedState = new String("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w w");
		assertTrue("States didn't match", testState.toString().equals(expectedState));
	}

	public void GameStateToStringTest2(){
		GameState testState = LNUtil.getGameState("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w w a3-c5");
		String expectedState = new String("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w w a3-c5");
		assertTrue("States didn't match", testState.toString().equals(expectedState));
	}
}
