exists :: String -> Bool
exists s = if not $ null s then True else False

map :: (a->Bool) -> [a] -> Bool
map f (x:_) = f x
