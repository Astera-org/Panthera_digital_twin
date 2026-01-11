// File loader utility class
export class FileLoader {
    constructor() {
        // Can add file loading related configuration here
    }

    async loadFile(file) {
        return new Promise((resolve, reject) => {
            const reader = new FileReader();
            reader.onload = (e) => resolve(e.target.result);
            reader.onerror = reject;
            reader.readAsText(file);
        });
    }
}

