mkdir ~/.ssh/

cd
cd .ssh

ssh-keygen -b 2048 -t rsa -f /tmp/sshkey -q -N ""
ssh-copy-id -i /tmp/sshkey ros@jetson

mv /tmp/sshkey ~/.ssh/sshkey
mv /tmp/sshkey.pub ~/.ssh/sshkey.pub

ssh-add -D
ssh-add sshkey

ssh-keyscan jetson >> ~/.ssh/known_hosts
