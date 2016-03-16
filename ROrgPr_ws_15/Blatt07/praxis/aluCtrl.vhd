library ieee;
use ieee.std_logic_1164.all;

entity aluCtrl is
	port(aluOp : in std_logic_vector(1 downto 0);
		 f : in std_logic_vector(5 downto 0);
		 operation : out std_logic_vector(3 downto 0));
end aluCtrl;

architecture behavioral of aluCtrl is
begin

	operation(3) <= aluOp(0) and not aluOp(0);
	operation(2) <= (aluOp(0) or (aluOp(1) and f(1)));
	operation(1) <= (not aluOp(1) or not f(2));
	operation(0) <= (aluOp(1) and (f(3) or f(0)));

end behavioral;
