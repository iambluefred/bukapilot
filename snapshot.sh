BRANCHES=(integration ports readme flip tempfix)

git checkout snapshot

for b in "${BRANCHES[@]}"
do
    git fetch kommu $b:$b
done

git fetch kommu base:base
git reset base --hard

for b in "${BRANCHES[@]}"
do
    git merge --squash $b && git commit --message "Merge '$b'"
done
