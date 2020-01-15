grep -Po "(?<=\[)[^\]]*?[^\\\\](?=\]($|[^:\[\(]))" $1 | env LC_COLLATE=C sort | uniq | while read -r line
do
    echo [$line]: $line
done
