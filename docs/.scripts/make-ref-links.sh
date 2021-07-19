echo [TOC]
echo
grep -Po "(?<=\[)[^\]]*?[^\\\\](?=\]($|[^:\[\(]))" $1 | env LC_COLLATE=C sort | uniq | while read -r line
do
    echo [$line]: @ref $line
done
