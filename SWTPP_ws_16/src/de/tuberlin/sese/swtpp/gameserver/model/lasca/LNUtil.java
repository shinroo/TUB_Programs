package de.tuberlin.sese.swtpp.gameserver.model.lasca;

/*
 * Parses Lasca Notation and returns correct objects
 */
public class LNUtil {

	public static String startingBoard = new String("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w w");

	public static GameState getGameState(String state){
		String[] gameInfo = state.split(" ");
		return new GameState(gameInfo[0], gameInfo[1]);
	}
}
