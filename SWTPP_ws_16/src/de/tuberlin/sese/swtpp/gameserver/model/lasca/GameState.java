package de.tuberlin.sese.swtpp.gameserver.model.lasca;

import java.io.Serializable;

public class GameState implements Serializable{

	private static final long serialVersionUID = -7102912345236150705L;
	
	public String board;
	public String player;
	public String lastMove;
	public boolean whiteWon;
	public boolean blackWon;

	public GameState(String board, String player, String lastMove){
		this.board = board;
		this.player = player;
		this.lastMove = lastMove;
		this.whiteWon = false;
		this.blackWon = false;
	}

	public GameState(String board, String player){
		this.board = board;
		this.player = player;
		this.lastMove = "";
		this.whiteWon = false;
		this.blackWon = false;
	}
	
	public String toString(){
		if(lastMove.equals("")){
			return new String(board +" "+ player);
		}else{
			return new String(board +" "+ player +" "+ lastMove);
		}
	}
	
	public boolean someoneWon(){
		return whiteWon || blackWon;
	}
}
