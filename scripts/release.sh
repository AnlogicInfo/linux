echo "${CI_COMMIT_TAG} ${CI_COMMIT_SHA}" > version
tar -cjf ${CI_COMMIT_BRANCH}-${CI_COMMIT_TAG}.tar.bz2 ./*
