package de.tuberlin.sese.swtpp.gameserver.test.lasca;

import static org.junit.Assert.*;

import org.junit.Test;

import de.tuberlin.sese.swtpp.gameserver.model.lasca.Field;

public class FieldTest {
	@Test
	public void isOfficerTest1(){
		Field testField = new Field(1, 1);
		testField.stones = "wB";
		assertFalse("Rob's whole life is over", testField.isOfficer());
	}

	@Test
	public void isOfficerTest2(){
		Field testField = new Field(1, 1);
		testField.stones = "BB";
		assertTrue("Rob's whole life is over", testField.isOfficer());
	}

	@Test
	public void isOfficerTest3(){
		Field testField = new Field(1, 1);
		testField.stones = "W";
		assertTrue("Rob's whole life is over", testField.isOfficer());
	}

	@Test
	public void isOfficerTest4(){
		Field testField = new Field(1, 1);
		testField.stones = "w";
		assertFalse("Rob's whole life is over", testField.isOfficer());
	}
	
	@Test
	public void isWhiteTest(){
		Field testField = new Field(1, 1);
		testField.stones = "W";
		assertTrue("Stick a pencil in your hair, man", testField.isWhite());
	}

	@Test
	public void isWhiteTest2(){
		Field testField = new Field(1, 1);
		testField.stones = "bwBwbB";
		assertFalse("Stick a pencil in your hair, man", testField.isWhite());
	}

	@Test
	public void isBlackTest(){
		Field testField = new Field(1, 1);
		testField.stones = "Bwb";
		assertTrue("Stick a pencil in your hair, man", testField.isBlack());
	}
}