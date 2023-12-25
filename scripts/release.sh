echo "${CI_COMMIT_TAG} ${CI_COMMIT_SHA}" > version
tar -cjf ${PROJECT_BRANCH_NAME}-${CI_COMMIT_TAG}.tar.bz2 ./*
