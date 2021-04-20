OUTPUT="./docs/js/birds.wasm.js"
INPUT="./source/unity.build.birds.lib.cpp"

PRE_JS="./docs/build/pre.js"
POST_JS="./docs/build/post.js"

script_dir=$(dirname "$0")

pushd $script_dir
cd ../../

emcc --bind \
     -O3 \
     -s ASSERTIONS=1 \
     -s WASM=1 \
     -s BINARYEN_ASYNC_COMPILATION=0 \
     -s SINGLE_FILE=1 \
     -s INITIAL_MEMORY=65536000 \
     -o $OUTPUT \
     $INPUT \
     -I ./include \
     -I ../cpp.algorithms/include \
     -I ../cpp.algorithms/3rdparty/folly \
     -DFOLLY_NO_CONFIG \
     -I /usr/local/Cellar/boost/1.65.1/include/ \
     -I ../cpp.algorithms/3rdparty/Sprout \
     -I ../cpp.algorithms/3rdparty/json/include \
     -DNDEBUG \
     -std=c++17 \
     -fconstexpr-steps=100000000 \
     -fconstexpr-depth=1000

cat $PRE_JS > /tmp/newfile
cat $OUTPUT >> /tmp/newfile
cp /tmp/newfile $OUTPUT

# there is a bug with '--post-js post.js' so we do it manually:
cat $POST_JS >> $OUTPUT

popd
