package de.tuberlin.sese.swtpp.gameserver.model.lasca;

import java.io.Serializable;
import java.util.ArrayList;

public class Board implements Serializable{

	private static final long serialVersionUID = -7102992505236150705L;

	private Field[][] fields = new Field[7][7];
	private GameState state;
	private String executedMove;
	private String nextPlayer;

	public Board(GameState gameState) {

		for (int i = 0; i < 7; i++) {
			for (int j = 0; j < 7; j++) {
				fields[i][j] = new Field(i, j);
			}
		}
		state = gameState;
		nextPlayer = new String("");
		executedMove = new String("");
		updateBoard();
	}

	public boolean attemptMove(Move move){
		// 1. Is move in list of allowed moves?
		boolean legalMove = false;
		for (Move m : getPossibleMoves()) {
			if(m.toString().equals(move.toString()))
				legalMove = true;
		}

		if(!legalMove){
			return false;
		}
		
		// 2. Execute!!!
		if(checkMoveType(move) == 0){
			performJump(move);
		}else{
			performTake(move);
		}

		checkPromotion(move);
		return true;
	}
	
	private void checkPromotion(Move move){
		if(move.end.y == 0 || move.end.y == 6){
			String officer = move.end.stones.substring(0, 1).toUpperCase();
			move.end.stones = officer.concat(move.end.stones.substring(1));
		}
	}

	public ArrayList<Move> getPossibleMoves() {
		ArrayList<Field> currentPlayerFields = getCurrentPlayerFields();
		ArrayList<Move> takes = new ArrayList<>();
		ArrayList<Move> jumps = new ArrayList<>();
		for (Field f : currentPlayerFields) {
			ArrayList<Field> legalFields = getLegalFields(f);
			for (Field nF : legalFields) { // neighbouring field
				Move move = new Move(f, nF);
				int moveType = checkNeihbouringMoveType(move);
				if (moveType == 0) {
					jumps.add(move);
				} else if (moveType == 1) {
					move.end = getTakeEndPosition(move);
					takes.add(move);
				} else {
					continue;
				}
			}
		}
		if (takes.isEmpty()) {
			return jumps;
		} else {
			return takes;
		} // TODO
	}

	public ArrayList<Move> getPossibleTakesForField(Field f) {
		ArrayList<Move> takes = new ArrayList<>();
		ArrayList<Field> legalFields = getLegalFields(f);
		for (Field nF : legalFields) { // neighbouring field
			Move move = new Move(f, nF);
			int moveType = checkNeihbouringMoveType(move);
			if (moveType == 1) { 
				move.end = getTakeEndPosition(move);
				takes.add(move);
			} else {
				continue;
			}
		}
		return takes;
	}

	// 0=jump, 1=take, 2=no movity
	public int checkNeihbouringMoveType(Move move) {
		if (move.end.stones.equals("")) {
			return 0;
		} else if (move.start.isWhite() != move.end.isWhite()) {
			// check for take
			Field endPosition = getTakeEndPosition(move);
			if (endPosition != null && endPosition.isFree()) {
				return 1;
			} else {
				return 2;
			}
		} else {
			return 2;
		}
	}

	public int checkMoveType(Move move) {
		if (move.isTake()) {
			return 1;
		} else {
			return 0;
		}
	}

	public Field getTakeEndPosition(Move move) {
		int y = move.end.y * 2 - move.start.y;
		int x = move.end.x * 2 - move.start.x;
		return getFieldByIndex(x, y);
	}

	public Field getTakeMiddlePosition(Move move) {
		int y = (move.end.y + move.start.y) / 2;
		int x = (move.end.x + move.start.x) / 2;
		return getFieldByIndex(x, y);
	}

	public Move FENToMove(String s){
		String[] split = s.split("-");
		return new Move(fields[getIndex(split[0].substring(0, 1))][getIndex(split[0].substring(1))],
			fields[getIndex(split[1].substring(0, 1))][getIndex(split[1].substring(1))]);
	}

	public ArrayList<Field> getCurrentPlayerFields() {
		ArrayList<Field> currentPlayerFields = new ArrayList<>();
		String f = new String();
		int x, y;
		for (y = 6; y >= 0; y--) {
			if (y % 2 == 0)
				x = 0;
			else
				x = 1;
			for (; x < 7; x += 2) {
				f = fields[x][y].stones;
				if (f.equals(""))
					continue;
				if (state.player.equals(f.substring(0, 1).toLowerCase()))
					currentPlayerFields.add(fields[x][y]);
			}
		}
		return currentPlayerFields;
	}

	public void updateBoard() {
		String parsedState = state.board.replaceAll("/", ",");
		String[] stones = parsedState.split(",", 25);
		int x, y, stonesIndex = 0;
		for (y = 6; y >= 0; y--) {
			if (y % 2 == 0)
				x = 0;
			else
				x = 1;
			for (; x < 7; x += 2) {
				this.fields[x][y].stones = stones[stonesIndex];
				stonesIndex++;
			}
		}
	}

	public GameState getBoardState() {
		return this.state;
	}

	public Field getFieldByIndex(int x, int y) {
		try {
			return fields[x][y];
		} catch (ArrayIndexOutOfBoundsException e) {
			return null;
		}
	}

	public ArrayList<Field> getLegalFields(Field f) {
		ArrayList<Field> neighbours = new ArrayList<>();

		if (f.isWhite() || f.isOfficer()) {

			// top-right
			if (getFieldByIndex(f.x + 1, f.y + 1) != null)
				neighbours.add(getFieldByIndex(f.x + 1, f.y + 1));

			// top-left
			if (getFieldByIndex(f.x - 1, f.y + 1) != null)
				neighbours.add(getFieldByIndex(f.x - 1, f.y + 1));
		}

		if (f.isBlack() || f.isOfficer()) {

			// bottom-right
			if (getFieldByIndex(f.x - 1, f.y - 1) != null)
				neighbours.add(getFieldByIndex(f.x - 1, f.y - 1));

			// bottom-left
			if (getFieldByIndex(f.x + 1, f.y - 1) != null)
				neighbours.add(getFieldByIndex(f.x + 1, f.y - 1));
		}

		return neighbours;
	}

	public GameState getFinalBoardState() {
		return this.getFinalizedBoard();
	}

	private GameState getFinalizedBoard() {
		String newState = new String("");
		String currentRow = new String("");
		int x, y;
		for (y = 6; y >= 0; y--) {
			if (y % 2 == 0)
				x = 0;
			else
				x = 1;
			for (; x < 7; x += 2) {
				currentRow = currentRow + (fields[x][y].stones + ",");
			}
			newState = newState + currentRow.substring(0, currentRow.length() - 1) + "/";
			currentRow = "";
		}
		newState = newState.substring(0, newState.length() - 1);

		GameState nextGameState = new GameState(newState, nextPlayer, executedMove);
		
		setWinner(nextGameState);
		return nextGameState;
	}
	
	private void setWinner(GameState state){
		Board nextBoard = new Board(state);
		boolean lost = nextBoard.getPossibleMoves().isEmpty();
		if(lost){
			if(nextBoard.state.player == "w"){
				state.blackWon = true;
			}else{
				state.whiteWon = true;
			}
		}else{
			;
		}
	}

	private void performJump(Move move) {
		// jump
		move.end.stones = move.start.stones;
		move.start.stones = "";
		nextPlayer = getOtherPlayer();
	}

	private void performTake(Move move) {
		// take
		Field middleField = getTakeMiddlePosition(move);
		String takenStone = middleField.stones.substring(0, 1);
		middleField.stones = middleField.stones.substring(1);
		move.end.stones = move.start.stones + takenStone;
		move.start.stones = "";
		if(getPossibleTakesForField(move.end).isEmpty()) {
			nextPlayer = getOtherPlayer();
		}else{
			nextPlayer = state.player;
			this.executedMove = move.toString();
		}
	}
/*
	public int getIndex(String s) throws RuntimeException {
		if (s.equals("a") || s.equals("1"))
			return 0;
		else if (s.equals("b") || s.equals("2"))
			return 1;
		else if (s.equals("c") || s.equals("3"))
			return 2;
		else if (s.equals("d") || s.equals("4"))
			return 3;
		else if (s.equals("e") || s.equals("5"))
			return 4;
		else if (s.equals("f") || s.equals("6"))
			return 5;
		else if (s.equals("g") || s.equals("7"))
			return 6;
		else
			throw new RuntimeException("Kurwa");
	}
	*/
	public int getIndex(String s) throws RuntimeException {
		int index = Character.getNumericValue(s.charAt(0));
		if(index <=9){
			return index-1;
		}else if(index <= 17){
			return index-10;
		}else{
			throw new RuntimeException("Wrong index");
		}
	}
	
	private String getOtherPlayer(){
		if (state.player.equals("w")) {
			return "b";
		} else {
			return "w";
		}
		
	}
}
