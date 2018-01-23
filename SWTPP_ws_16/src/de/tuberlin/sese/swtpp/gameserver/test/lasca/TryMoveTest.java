package de.tuberlin.sese.swtpp.gameserver.test.lasca;

import static org.junit.Assert.assertEquals;

import org.junit.Before;
import org.junit.Test;

import de.tuberlin.sese.swtpp.gameserver.control.GameController;
import de.tuberlin.sese.swtpp.gameserver.model.Move;
import de.tuberlin.sese.swtpp.gameserver.model.Player;
import de.tuberlin.sese.swtpp.gameserver.model.User;
import de.tuberlin.sese.swtpp.gameserver.model.lasca.Field;
import de.tuberlin.sese.swtpp.gameserver.model.lasca.LNUtil;
import de.tuberlin.sese.swtpp.gameserver.model.lasca.LascaGame;

public class TryMoveTest {

	User user1 = new User("Alice", "alice");
	User user2 = new User("Bob", "bob");
	
	Player whitePlayer = null;
	Player blackPlayer = null;
	LascaGame game = null;
	GameController controller;
	
	@Before
	public void setUp() throws Exception {
		controller = GameController.getInstance();
		controller.clear();
		
		int gameID = controller.startGame(user1, "");
		
		game = (LascaGame) controller.getGame(gameID);
		whitePlayer = game.getPlayer(user1);

	}
	
	
	public void startGame(String initialBoard, boolean whiteNext) {
		controller.joinGame(user2);		
		blackPlayer = game.getPlayer(user2);
		
		game.setState(initialBoard);
		game.setNextPlayer(whiteNext? whitePlayer:blackPlayer);
	}
	
	public void assertMove(String move, boolean white, boolean expectedResult) {
		if (white)
			assertEquals(game.tryMove(move, whitePlayer), expectedResult);
		else 
			assertEquals(game.tryMove(move, blackPlayer), expectedResult);
	}
	
	public void assertGameState(String expectedBoard, boolean whiteNext, boolean finished, boolean whiteWon) {
		assertEquals(game.getState(), expectedBoard);
		assertEquals(game.isWhiteNext(), whiteNext);

		assertEquals(game.isFinished(), finished);
		if (game.isFinished()) {
			assertEquals(whitePlayer.isWinner(), whiteWon);
			assertEquals(blackPlayer.isWinner(), !whiteWon);
		}
	}
	
	/*******************************************
	 * !!!!!!!!! To be implemented !!!!!!!!!!!!
	 *******************************************/
	
	@Test
	public void exampleTest() {
		startGame("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w", true);
		assertMove("a3-b4", true, true);
		assertGameState("b,b,b,b/b,b,b/b,b,b,b/w,,/,w,w,w/w,w,w/w,w,w,w", false, false, false);
	}

	//TODO: implement test cases of same kind as example here
	
	@Test
	public void initialThreeTurns(){
		startGame("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w", true);
		assertMove("c3-b4", true, true);
		assertGameState("b,b,b,b/b,b,b/b,b,b,b/w,,/w,,w,w/w,w,w/w,w,w,w", false, false, false);
		assertMove("a5-c3", false, true);
		assertGameState("b,b,b,b/b,b,b/,b,b,b/,,/w,bw,w,w/w,w,w/w,w,w,w", true, false, false);
	}
	
	@Test
	public void doubleTakeTest1(){
		startGame("b,,,/w,,/,,,/,w,/,,,/,,/,,,", false);
		assertMove("a7-c5", false, true);
		assertGameState(",,,/,,/,bw,,/,w,/,,,/,,/,,,", false, false, false);
		assertMove("c5-e3", false, true);
		assertGameState(",,,/,,/,,,/,,/,,bww,/,,/,,,", true, true, false);
	}
	
	@Test
	public void promotionTest1(){
		startGame(",,,/w,,/,,,/,,/,,,/b,,/,,,", true);
		assertMove("b6-a7", true, true);
		assertGameState("W,,,/,,/,,,/,,/,,,/b,,/,,,", false, false, false);
	}

	@Test
	public void promotionTest2(){
		startGame(",,,/wbbb,,/,,,/,,/,,,/b,,/,,,", true);
		assertMove("b6-a7", true, true);
		assertGameState("Wbbb,,,/,,/,,,/,,/,,,/b,,/,,,", false, false, false);
	}

	@Test
	public void promotionTest3(){
		startGame(",,,/bbb,,/,w,,/,,/,,,/b,,/,,,", true);
		assertMove("c5-a7", true, true);
		assertGameState("Wb,,,/bb,,/,,,/,,/,,,/b,,/,,,", false, false, false);
	}
	
	@Test
	public void emptyBoard1(){
		startGame(",,,/b,,/,w,,/,,/,,,/,,/,,,", true);
		assertMove("c5-a7", true, true);
		assertGameState("Wb,,,/,,/,,,/,,/,,,/,,/,,,", false, true, true);
	}
	
	@Test
	public void fullGameTest1(){
		startGame("b,b,b,b/b,b,b/b,b,b,b/,,/w,w,w,w/w,w,w/w,w,w,w", true);
		assertMove("a1-a3", true, false);
		assertMove("a2-z2", true, false);

		assertMove("a9-a0", true, false);
		assertMove("a9-z-1", true, false);

		assertMove("c3-b4", true, true);
		assertMove("a5-c3", false, true);

		assertMove("d2-b4", true, true);
		assertMove("c5-d4", false, true);

		assertMove("e3-c5", true, true);
		assertMove("b6-d4", false, true);

		assertMove("e1-d2", true, true);
		assertMove("e5-f4", false, true);

		assertMove("g3-e5", true, true);
		assertMove("d6-f4", false, true);

		assertMove("b4-d6", true, true);
		assertMove("e7-c5", false, true);

		assertMove("d2-e3", true, true);
		assertMove("f4-d2", false, true);
		

		assertMove("c1-e3", true, true);
		assertMove("c5-b4", false, true);

		assertMove("c3-a5", true, true);
		assertMove("g5-f4", false, true);

		assertMove("e3-g5", true, true);
		assertMove("g5-e7", true, true);
		assertMove("g7-f6", false, true);

		assertMove("e7-c5", true, true);
		assertMove("c5-e3", true, true);
		assertMove("e5-c3", false, true);
		assertMove("c3-e1", false, true);

		assertMove("e3-d4", true, true);
		assertMove("e1-c3", false, true);
		assertMove("c3-e5", false, true);

		assertMove("f2-e3", true, true);
		assertMove("d4-f2", false, true);

		assertMove("g1-e3", true, true);
		assertMove("f6-g5", false, true);

		assertMove("e3-d4", true, true);
		assertMove("e5-c3", false, true);

		assertMove("b4-c5", true, true);
		assertMove("d6-b4", false, true);

		assertMove("a3-c5", true, true);
		assertMove("c3-d2", false, true);

		assertMove("c5-b6", true, true);
		assertMove("a7-c5", false, true);
		assertMove("c5-a3", false, true);
		assertMove("a3-c1", false, true);

		assertMove("a1-b2", true, true);
		assertMove("c1-a3", false, true);
		
		assertGameState(",b,,/b,,/wb,,,b/,b,/Bwwww,,,/,BwwwWw,bbbbw/,,,", true, true, false);
	}
	
	@Test
	public void fullGameTest2(){
		startGame("b,b,,/b,wbb,wwbb/b,wbb,,/,,/w,ww,,wb/,w,/w,,,w", false);
		assertMove("b6-d4", false, true);
		assertMove("d4-b2", false, true);
		assertMove("c3-d4", true, true);
		//System.out.println(game.getState());
		assertMove("c7-e5", false, true);
		//System.out.println(game.getState());
		assertMove("e5-c3", false, true);
		//System.out.println(game.getState());
		assertMove("c3-e1", false, true);
		//System.out.println(game.getState());
	}
	
	@Test (expected=RuntimeException.class)
	public void catchExceptionTest(){
		Field field = new Field(10, 10);
		field.toString();
	}
}
