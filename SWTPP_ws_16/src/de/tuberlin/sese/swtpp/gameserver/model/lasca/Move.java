package de.tuberlin.sese.swtpp.gameserver.model.lasca;

public class Move {
	public Field start;
	public Field end;

	public Move(Field s, Field e){
		this.start = s;
		this.end = e;
	}
	public boolean isTake(){
		return (start.y%2 == end.y%2);
	}
	
	public String toString(){
		return new String(start.toString() +"-"+end.toString());
	}
}
