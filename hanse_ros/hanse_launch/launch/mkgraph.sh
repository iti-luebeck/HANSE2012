#!/bin/bash
(
echo "digraph {"
for file in *.launch; do
    base=$(basename $file .launch)
    for include in $(grep "launch)/launch" $file | sed 's/^.*\/\([a-z0-9_]*\)\.launch.*$/\1/'); do
        echo "$base -> $include"
    done
done
echo "}"
) | dot -Tpng > graph.png
