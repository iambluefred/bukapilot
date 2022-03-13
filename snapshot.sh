echo "[Usage] Add the --release flag to merge for release"
BRANCHES=(tool integration ports readme flip tempfix)
RELEASE_OPT=$@

for b in "${BRANCHES[@]}"
do
    git fetch origin $b:$b
done

git fetch origin base_new:base_new
git reset base_new --hard

for b in "${BRANCHES[@]}"
do
    if [ "$RELEASE_OPT" = "--release" ] && [ "$b" = "tool" ]; then
    	continue
    fi
    git merge --squash $b && git commit --message "Merge '$b'"
done
