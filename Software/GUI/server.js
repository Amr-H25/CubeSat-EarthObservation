const express = require('express');
const fs = require('fs');
const path = require('path');
const cors = require('cors');

const IMAGE_FOLDER = path.join(__dirname, 'images');
const IMAGE_EXTENSIONS = ['.jpg', '.jpeg', '.png', '.gif', '.bmp', '.webp'];

const app = express();
app.use(cors());

// Endpoint to get the latest image filename
app.get('/latest-image', (req, res) => {
    fs.readdir(IMAGE_FOLDER, (err, files) => {
        if (err) return res.status(500).json({ error: 'Cannot read image folder' });
        const images = files
            .filter(file => IMAGE_EXTENSIONS.includes(path.extname(file).toLowerCase()))
            .map(file => ({
                file,
                time: fs.statSync(path.join(IMAGE_FOLDER, file)).mtime.getTime()
            }))
            .sort((a, b) => b.time - a.time);
        if (images.length === 0) return res.status(404).json({ error: 'No images found' });
        res.json({ filename: images[0].file });
    });
});

// Serve images statically
app.use('/images', express.static(IMAGE_FOLDER));

// Start server
const PORT = 3001;
app.listen(PORT, () => {
    console.log(`Image server running at http://localhost:${PORT}`);
});
