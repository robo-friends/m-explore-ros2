file_name=$1
url=$2
md5=$3

# Download the file if it doesn't exist
if [ ! -f $file_name ]; then
    wget $url -O $file_name
fi

# Check the MD5 sum of the file
echo "Checking MD5 sum of $file_name"
md5sum -c <<<"$md5 $file_name"
if [ $? -ne 0 ]; then
    echo "MD5 sum of $file_name does not match. Downloading it again"
    wget $url -O $file_name
    md5sum -c <<<"$md5 $file_name"
    if [ $? -ne 0 ]; then
        echo "MD5 sum of $file_name still does not match. Aborting."
        exit 1
    fi
fi