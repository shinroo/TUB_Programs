library ieee;
use ieee.std_logic_1164.all;

entity mipsCtrl is
	port(op : in std_logic_vector(5 downto 0);
		 regDst : out std_logic;
		 branch : out std_logic;
		 memRead : out std_logic;
		 memToReg : out std_logic;
		 aluOp : out std_logic_vector(1 downto 0);
		 memWrite : out std_logic;
		 aluSrc : out std_logic;
		 regWrite : out std_logic);
end mipsCtrl;

architecture structural of mipsCtrl is
signal and1, and2, and3, and4 : std_logic;
begin

	and1 <= not op(0) and not op(1) and not op(2) and not op(3) and not op(4) and not op(5);
	and2 <= op(0) and op(1) and not op(2) and not op(3) and not op(4) and op(5);
	and3 <= op(0) and op(1) and not op(2) and op(3) and not op(4) and op(5);
	and4 <= not op(0) and not op(1) and op(2) and not op(3) and not op(4) and not op(5);

	regDst <= and1;
	aluSrc <= and2 or and3;
	memToReg <= and2;
	regWrite <= and1 or and2;
	memRead <= and2;
	memWrite <= and3;
	branch <= and4;
	aluOp(1) <= and1;
	aluOp(0) <= and4;

end structural;
