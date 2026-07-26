#!/usr/bin/env bash

FAIL=0

# -I skips binary files. Without it, LFS-materialised binaries that happen to start with a
# shebang (system/hardware/tici/updater) make grep emit "Binary file X matches" instead of the
# matched line, which then fails the format filter and reports a bogus invalid shebang. This only
# shows up once `git lfs pull` has run — i.e. on any correctly set-up checkout.
if grep -I '^#!.*python' $@ | grep -v '#!/usr/bin/env python3$'; then
  echo -e "Invalid shebang! Must use '#!/usr/bin/env python3'\n"
  FAIL=1
fi

if grep -I '^#!.*bash' $@ | grep -v '#!/usr/bin/env bash$'; then
  echo -e "Invalid shebang! Must use '#!/usr/bin/env bash'"
  FAIL=1
fi

exit $FAIL
