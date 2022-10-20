#!/bin/bash
# update compressed spiffs files
cd data_uncompressed
mkdir -p ../data
for f in *; do
  ext=${f##*.}
  if [[ " html htm js css ico " == *" $ext "* ]]; then zip=".gz"; else zip=""; fi
  if [ $f -nt ../data/$f$zip ]; then
    cp $f ../data/ && [ $zip ] && gzip -f ../data/$f
  fi
done

# git revision macro
#echo -DSRC_REV=\\\"$(git rev-parse HEAD)\\\"
echo -DSRC_REVISION=\\\"$(git --no-pager show --date=short --format="%ad" --name-only | head -n1)_$(git --no-pager describe --tags --always --dirty)\\\"
