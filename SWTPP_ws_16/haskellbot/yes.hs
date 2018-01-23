-- Take FEN string
splitOnSlash :: [a] -> [[a]]
splitOnSlash x = splitOn "/" x

commaListSmall :: [[Char]]
commaListSmall = [",",",",",",",",",",","]

commaListBig :: [[Char]]
commaListBig = [",",",",",",",",",",",",",",",",",",",",",",",",",",",",",",",",",",","]

getFlat :: [a] -> [a] -> [a]
getFlat x y = concat $ interleave x y

getFinal :: [a] -> [a]
getFinal x = interleave (getFlat (interleave (splitOnSlash x) commaListSmall)) commaListBig 
